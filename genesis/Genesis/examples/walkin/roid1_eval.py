import argparse
import os
import pickle
import sys
import time
import numpy as np
import torch
from roid1_env import Roid1Env
from rsl_rl.runners import OnPolicyRunner
import genesis as gs
import redis
import signal

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout
from PySide6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

plt.switch_backend("QtAgg")  # 高速なQtバックエンドを使用

# Redis の設定
r = redis.Redis(host='localhost', port=6379, decode_responses=True)
meridis_size = 90  # meridisの要素数

# `meridis` を初期化
if not r.exists("meridis"):
    r.rpush("meridis", *[0] * meridis_size)
    print(f"Initialized Redis list 'meridis' with {meridis_size} elements.")

# `app` をグローバルスコープに定義
app = None  

# ユーザーが定義する関節名と `meridis` のインデックス
joint_to_meridis = {
    "l_shoulder_p": 23,
    "l_hipjoint_r": 33,
    "l_hipjoint_p": 35,
    "l_knee_p": 37,
    "l_ankle_p": 39,
    "l_ankle_r": 41,
    "r_shoulder_p": 53,
    "r_hipjoint_r": 63,
    "r_hipjoint_p": 65,
    "r_knee_p": 67,
    "r_ankle_p": 69,
    "r_ankle_r": 71
}

def signal_handler(sig, frame):
    global app  # `app` をグローバル変数として参照
    print("\n[INFO] Ctrl+C を検出しました。アプリケーションを終了します。")
    if app is not None:
        app.quit()
        QTimer.singleShot(100, lambda: sys.exit(0))  # 100ms 後に確実に終了

class ClockGauge(QMainWindow):
    def __init__(self, joint_names):
        super().__init__()

        self.setWindowTitle("Joint Angle Gauge")
        self.setGeometry(100, 100, 500, 500)  
        self.setStyleSheet("background-color: #2E2E2E;")  

        self.joint_names = joint_names
        self.num_joints = len(joint_names)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        layout = QGridLayout(self.central_widget)

        self.figures, self.axes, self.canvas_list, self.needle_lines, self.backgrounds = [], [], [], [], []
        grid_size = int(np.ceil(np.sqrt(self.num_joints)))

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
            self.backgrounds.append(canvas.copy_from_bbox(ax.bbox))

    def update_plot(self, joint_angles):
        theta = np.mod(joint_angles + np.pi, 2*np.pi) - np.pi

        for i, angle in enumerate(theta):
            x, y = 0.25 * np.sin(angle), 0.25 * np.cos(angle)
            self.needle_lines[i].set_data([0, x], [0, y])

        for i, ax in enumerate(self.axes):
            self.canvas_list[i].restore_region(self.backgrounds[i])
            ax.draw_artist(self.needle_lines[i])
            self.canvas_list[i].blit(ax.bbox)
            self.canvas_list[i].flush_events()

def main():
    global app  # `app` をグローバルスコープに設定

    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="roid1-walking")
    parser.add_argument("--ckpt", type=int, default=100)
    args = parser.parse_args()

    gs.init()
    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env = Roid1Env(num_envs=1, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg, show_viewer=True)
    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")
    runner.load(os.path.join(log_dir, f"model_{args.ckpt}.pt"))
    policy = runner.get_inference_policy(device="cuda:0")

    obs, _ = env.reset()
    joint_names = env.env_cfg["dof_names"]

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal_handler)

    plot_window = ClockGauge(joint_names)
    plot_window.show()

    def update():
        nonlocal obs
        actions = policy(obs)
        obs, _, _, dones, _ = env.step(actions)
        joint_positions = env.dof_pos.to("cuda").squeeze().cpu().numpy().reshape(-1)

        if not r.exists("meridis"):
            r.rpush("meridis", *[0] * meridis_size)
            print(f"Initialized Redis list 'meridis' with {meridis_size} elements.")

        for joint_name, index in joint_to_meridis.items():
            if joint_name in joint_names:
                joint_idx = joint_names.index(joint_name)
                value = round(np.degrees(float(joint_positions[joint_idx])), 2)
                r.lset("meridis", index, value)
                #print(f"Setting Redis meridis[{index}] = {value} degrees")

        plot_window.update_plot(joint_positions)

    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(20)

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
