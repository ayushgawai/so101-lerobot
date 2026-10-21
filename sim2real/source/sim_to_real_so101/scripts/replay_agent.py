# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Replays a joint-target action log recorded by keyboard_agent.py's --action_log.

Open-loop: steps the environment through the exact recorded per-step joint
targets, in order, with the scene visible. No `lerobot` dependency -- reads
the plain .npz written by keyboard_agent.py (see docs/isaac-sim-windows-guide.md
section 7).
"""
import argparse
import os

# Import h5py before Isaac Sim/Kit boots: Kit loads its own native libraries
# (e.g. z.dll) that can shadow h5py's bundled HDF5 DLLs on Windows, causing
# "DLL load failed while importing _errors" once isaaclab_tasks imports h5py
# later. Importing it first locks in h5py's own DLLs before the conflict can occur.
import h5py  # noqa: F401

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Isaac Lab SO-101 action-log replay agent.")
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--num_envs", type=int, default=None, help="Number of environments to simulate."
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--action_log",
    type=str,
    required=True,
    help="Path to an episode_NNN.npz file written by keyboard_agent.py's --action_log.",
)
parser.add_argument(
    "--num_repeats",
    type=int,
    default=1,
    help="How many times to replay the log before exiting (env resets between repeats).",
)
parser.add_argument("--seed", type=int, default=101, help="Environment seed")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# always enable cameras (harmless if unused, matches the other scripts)
args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import gymnasium as gym
import numpy as np
import torch
from tqdm import tqdm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
import sim_to_real_so101.tasks  # noqa: F401
from sim_to_real_so101.tasks.task_env_cfg import apply_rtx_translucency_settings
from sim_to_real_so101.utils.keyboard import KeyboardControl, JointJogKeyboardControl


def main():

    keyboard_control = KeyboardControl()

    log = np.load(args_cli.action_log)
    logged_actions = torch.as_tensor(log["actions"], dtype=torch.float32)  # (T, 6)
    logged_joint_names = list(log["joint_names"])
    if logged_joint_names != JointJogKeyboardControl.JOINT_ORDER:
        raise ValueError(
            f"Action log joint order {logged_joint_names} does not match the expected "
            f"order {JointJogKeyboardControl.JOINT_ORDER} -- was this log written by a "
            "different version of keyboard_agent.py?"
        )
    print(f"[INFO]: Loaded {logged_actions.shape[0]} steps from {args_cli.action_log}")

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.seed = args_cli.seed
    env = gym.make(args_cli.task, cfg=env_cfg)
    # applied here (not in EnvCfg.__post_init__) to dodge a gym.make()-time
    # race against the RTX renderer extension's own registration -- see
    # apply_rtx_translucency_settings()'s docstring
    apply_rtx_translucency_settings()

    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    print(f"[INFO]: Click 'R' to restart the replay from the beginning")

    logged_actions = logged_actions.to(env.unwrapped.device)
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)

    env.reset()

    for repeat in range(args_cli.num_repeats):
        if not simulation_app.is_running():
            break

        print(f"[INFO]: Replay {repeat + 1}/{args_cli.num_repeats}")
        step = 0
        pbar = tqdm(total=logged_actions.shape[0], desc="Replaying", unit="step")

        while step < logged_actions.shape[0] and simulation_app.is_running():
            with torch.inference_mode():
                actions[:, :] = logged_actions[step]
                env.step(actions)
                step += 1
                pbar.update(1)

                if keyboard_control.reset_world:
                    keyboard_control.reset_world = False
                    env.reset()
                    step = 0
                    pbar.reset()

        pbar.close()

        if repeat + 1 < args_cli.num_repeats and simulation_app.is_running():
            env.reset()

    print(f"[INFO]: Replay finished. Window stays open -- close it or Ctrl+C to exit.")
    while simulation_app.is_running():
        simulation_app.update()

    env.close()


if __name__ == "__main__":

    main()

    while True:
        simulation_app.update()

    simulation_app.close()
