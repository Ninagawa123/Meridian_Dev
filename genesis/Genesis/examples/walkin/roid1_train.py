import argparse
import os
import pickle
import shutil

import torch
from roid1_env import Roid1Env
from rsl_rl.runners import OnPolicyRunner

import genesis as gs

def get_train_cfg(exp_name, max_iterations):

    train_cfg_dict = {
        "algorithm": {
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 1.0,
        },
        "runner": {
            "algorithm_class_name": "PPO",
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "num_steps_per_env": 24,
            "policy_class_name": "ActorCritic",
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
            "runner_class_name": "runner_class_name",
            "save_interval": 100,
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs():
    env_cfg = {
        "num_actions": 12,      # pytorch limit
        # urdfで　hipjoint_yをfixedにした場合、shourlder_r を制御するとバランスをとりやすい
        # urdfで　hipjoint_yをrevoluteにした場合、shourlder_r をコメント無効にする(python行列計算の制約)
        # joint/link names
        "default_joint_angles": {  # [rad]
            "l_shoulder_p": 0.0,
            #"l_shoulder_r": 0.0,#
            #"l_elbow_p":  0.0,
            #"l_hipjoint_y": 0.0,
            "l_hipjoint_r": 0.0,
            "l_hipjoint_p": -0.5,
            "l_knee_p":  1.0,
            "l_ankle_p": -0.5,
            "l_ankle_r": 0.0,
            "r_shoulder_p": 0.0,
            #"r_shoulder_r": 0.0,#         
            #"r_elbow_p":  0.0,
            #"r_hipjoint_y": 0.0,
            "r_hipjoint_r": 0.0,
            "r_hipjoint_p": -0.5,
            "r_knee_p":  1.0,
            "r_ankle_p":  -0.5,
            "r_ankle_r": 0.0,
        },
        "dof_names": [
            "l_shoulder_p",
            #"l_shoulder_r",#
            #"l_elbow_p",
            #"l_hipjoint_y",
            "l_hipjoint_r",
            "l_hipjoint_p",
            "l_knee_p",
            "l_ankle_p",
            "l_ankle_r",
            "r_shoulder_p",
            #"r_shoulder_r", #           
            #"r_elbow_p",
            #"r_hipjoint_y",
            "r_hipjoint_r",
            "r_hipjoint_p",
            "r_knee_p",
            "r_ankle_p",
            "r_ankle_r",
        ],
        # PD
        "kp": 20.0,
        "kd": 0.5,
        # termination
        "termination_if_roll_greater_than": 10,     # degree roll
        "termination_if_pitch_greater_than": 10,    # degree pitch
        # base pose
        "base_init_pos": [0.0, 0.0, 0.30],           # 初期位置 0.0, 0.0, 0.42
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],     # 初期姿勢 1.0, 0.0, 0.0, 0.0
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,
        "simulate_action_latency": True,
        "clip_actions": 100.0,

    }
    obs_cfg = {
        "num_obs": 45,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.300,    # 0.300
        "feet_height_target": 0.075,    # 0.075
        "reward_scales": {
            "tracking_lin_vel": 1.0,
            "tracking_ang_vel": 0.2,
            "lin_vel_z": -1.0,          # -1.0
            "base_height": -50.0,       # -50.0
            "action_rate": -0.005,
            "similar_to_default": -0.1,
        },
    }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.5, 0.5],
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="roid1-walking")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=100) # for training
    #parser.add_argument("--max_iterations", type=int, default=10)   # for test
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    env = Roid1Env(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# training
python examples/walking/go2_train.py
"""
