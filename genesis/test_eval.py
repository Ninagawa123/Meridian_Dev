import argparse
import os
import pickle
import sys
import time
import numpy as np
import torch
from go2_env import Go2Env
from rsl_rl.runners import OnPolicyRunner
import genesis as gs

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout
from PySide6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

plt.switch_backend("Qt5Agg")  # 高速なQtバックエンドを使用

class ClockGauge(QMainWindow):
    def __init__(self, joint_names):
        super().__init__()

        self.setWindowTitle("Joint Angle Gauge")
        self.setGeometry(100, 100, 500, 500)  # ウィンドウサイズ
        self.setStyleSheet("background-color: #2E2E2E;")  # 深いグレーの背景

        self.joint_names = joint_names
        self.num_joints = len(joint_names)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        layout = QGridLayout(self.central_widget)

        self.figures = []
        self.axes = []
        self.canvas_list = []
        self.needle_lines = []
        self.backgrounds = []

        grid_size = int(np.ceil(np.sqrt(self.num_joints)))  # 正方形に近い形で並べる

        for i, name in enumerate(self.joint_names):
            fig, ax = plt.subplots(figsize=(1.0, 1.0))
            fig.patch.set_facecolor("#2E2E2E")
            ax.set_facecolor("#2E2E2E")
            canvas = FigureCanvas(fig)
            layout.addWidget(canvas, i // grid_size, i % grid_size)

            ax.set_xlim(-0.3, 0.3)
            ax.set_ylim(-0.3, 0.3)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_aspect('equal')
            ax.set_title(name, fontsize=12, color="lightgray")

            circle = plt.Circle((0, 0), 0.25, color="lightgray", fill=False, linewidth=2)
            ax.add_patch(circle)

            line, = ax.plot([0, 0], [0, 0.25], color='lightgray', linewidth=1.5)

            self.figures.append(fig)
            self.axes.append(ax)
            self.canvas_list.append(canvas)
            self.needle_lines.append(line)

            self.backgrounds.append(canvas.copy_from_bbox(ax.bbox))  # 背景を保存

    def update_plot(self, joint_angles):
        theta = -joint_angles
        r = 0.25

        for i, angle in enumerate(theta):
            x = r * np.sin(angle)
            y = r * np.cos(angle)
            self.needle_lines[i].set_data([0, x], [0, y])

        for i, ax in enumerate(self.axes):
            self.canvas_list[i].restore_region(self.backgrounds[i])  # 背景復元
            ax.draw_artist(self.needle_lines[i])  # 針のみ再描画
            self.canvas_list[i].blit(ax.bbox)  # 部分描画
            self.canvas_list[i].flush_events()  # **画面を即時更新**

            
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking")
    parser.add_argument("--ckpt", type=int, default=100)
    args = parser.parse_args()

    gs.init()
    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env = Go2Env(num_envs=1, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg, show_viewer=True)
    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")
    runner.load(os.path.join(log_dir, f"model_{args.ckpt}.pt"))
    policy = runner.get_inference_policy(device="cuda:0")

    obs, _ = env.reset()
    joint_names = env.env_cfg["dof_names"]

    app = QApplication(sys.argv)
    plot_window = ClockGauge(joint_names)
    plot_window.show()

    target_fps = 50
    frame_time = 1.0 / target_fps

    def update():
        nonlocal obs
        if env.scene.viewer is None or getattr(env.scene.viewer, "closed", False):
            print("Genesis Viewer が閉じられたため、アプリケーションを終了します。")
            timer.stop()
            app.quit()
            return

        try:
            with torch.no_grad():
                actions = policy(obs)
                obs, _, _, dones, _ = env.step(actions)

                # **データの形状を強制的に整える**
                joint_positions = env.dof_pos.to("cuda").squeeze().cpu().numpy().reshape(-1)

                # **デバッグ用**
                print(f"Joint positions shape: {joint_positions.shape}")

                plot_window.update_plot(joint_positions)
                
                if dones.any():
                    obs, _ = env.reset()
        except Exception as e:
            print(f"[ERROR] {e}")
            timer.stop()
            app.quit()
            

    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(int(frame_time * 1000))

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
