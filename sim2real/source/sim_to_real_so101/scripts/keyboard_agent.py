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
"""Keyboard-jog teleop agent: no physical leader arm required.

Drives the SO-101's 6 joints directly with per-joint keyboard jogging
(held key -> continuous +/- delta on that joint's target position) instead
of a hardware leader arm over serial, for machines without one. See
docs/isaac-sim-windows-guide.md section 7 for why this exists.
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
parser = argparse.ArgumentParser(description="Isaac Lab SO-101 keyboard-jog teleop agent.")
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
    "--joint_step",
    type=float,
    default=0.01,
    help="Radians added/removed from a joint's target per physics step while its jog key is held.",
)
parser.add_argument(
    "--repo_id", type=str, default=None, help="Repository ID to store the dataset."
)
parser.add_argument(
    "--repo_root", type=str, default=None, help="Repository root to store the dataset."
)
parser.add_argument(
    "--action_log",
    type=str,
    default=None,
    help=(
        "Directory to save simple per-step joint-target logs (.npz, no `lerobot` dependency) "
        "each time recording is toggled off with 'S' (or implicitly via 'R'). Independent of "
        "--repo_id/--repo_root/--task_name -- replay these with replay_agent.py."
    ),
)
parser.add_argument(
    "--save_mp4",
    action="store_true",
    default=False,
    help="Save depth and RGB as mp4 videos.",
)
parser.add_argument(
    "--depth", action="store_true", default=False, help="Save depth as mp4 video."
)
parser.add_argument(
    "--instance_id_seg",
    action="store_true",
    default=False,
    help="Save instance id segmentation as mp4 video.",
)
parser.add_argument("--task_name", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=101, help="Environment seed")


# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# always enable cameras to record video
args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import gymnasium as gym
import numpy as np
import torch


import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
import sim_to_real_so101.tasks  # noqa: F401
from sim_to_real_so101.tasks.task_env_cfg import apply_rtx_translucency_settings
from sim_to_real_so101.utils.keyboard import JointJogKeyboardControl
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner

print_simulator_version_banner()

# LeRobotSO101Interface/LeRobotRecorder both import the `lerobot` pip package,
# which isn't installed by this repo's Isaac Lab pip install (only pulled in
# separately for hardware teleop). Deferred so plain keyboard jogging works
# without it -- only imported below if recording is actually requested.


def main():

    keyboard_control = JointJogKeyboardControl()

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    env_cfg.seed = args_cli.seed
    env = gym.make(args_cli.task, cfg=env_cfg)
    # applied here (not in EnvCfg.__post_init__) to dodge a gym.make()-time
    # race against the RTX renderer extension's own registration -- see
    # apply_rtx_translucency_settings()'s docstring
    apply_rtx_translucency_settings()

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    print_simulator_version_banner()
    print(f"[INFO]: Keyboard jog controls (hold to move, release to stop):")
    for joint in JointJogKeyboardControl.JOINT_ORDER:
        pos_key, neg_key = JointJogKeyboardControl.JOINT_KEYS[joint]
        print(f"    {joint:<12s}  {pos_key} (+)  /  {neg_key} (-)")
    print(f"[INFO]: Click 'R' to reset the world")
    print(f"[INFO]: Click 'S' to start/stop recording; 'R' will also stop recording")
    print(f"[INFO]: Click 'C' to cancel an in-progress recording")
    if args_cli.action_log:
        print(f"[INFO]: Action log enabled -> {args_cli.action_log} (one .npz per 'S' start/stop)")

    # reset environment
    env.reset()

    # cameras
    cameras = {}
    for obj in env.unwrapped.scene.keys():
        if obj.startswith("camera_"):
            camera_cfg = getattr(env.unwrapped.scene.cfg, obj)
            cameras[obj.replace("camera_", "")] = {
                "height": camera_cfg.height,
                "width": camera_cfg.width,
            }
            print(f"[INFO]: Found Camera: {obj.replace('camera_', '')}")
    if len(cameras) == 0:
        print(f"[Info]: No cameras found - videos will not be recorded")

    robot = env.unwrapped.scene["robot"]
    joint_indices = [robot.joint_names.index(name) for name in JointJogKeyboardControl.JOINT_ORDER]
    default_targets = robot.data.default_joint_pos[0, joint_indices].clone()
    joint_limits = robot.data.soft_joint_pos_limits[0, joint_indices]  # (6, 2) -> [lower, upper]
    targets = default_targets.clone()

    # Allocate action tensor
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)

    # Lightweight action-log recording (no `lerobot` dependency): reuses the
    # same 'S'/'R' recording toggle as the LeRobot dataset path below, but
    # just appends the per-step joint target and dumps a .npz on stop.
    action_log_mode = args_cli.action_log is not None
    action_log_buffer = []
    action_log_episode_index = 0
    was_recording = False
    if action_log_mode:
        os.makedirs(args_cli.action_log, exist_ok=True)

    # Recording dataset
    if all([args_cli.repo_id, args_cli.repo_root, args_cli.task_name]):
        recording_mode = True
    else:
        recording_mode = False

    if recording_mode:
        # Deferred: both pull in the `lerobot` pip package, not installed by
        # this repo's Isaac Lab pip install. Only needed when recording is
        # requested -- see the module-level note near the top of this file.
        from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface
        from sim_to_real_so101.utils.lerobot_recorder import LeRobotRecorder

        # Reused only for its sim<->real unit-conversion helpers (static joint-range
        # lookups, not a live connection) so recorded datasets stay in the same
        # degree-space convention lerobot_agent.py's hardware-teleop recordings use.
        # init_device()/connect() are intentionally not called -- no physical arm.
        robot_iface = LeRobotSO101Interface(
            device=env.unwrapped.device,
            port="",
            id="keyboard_jog",
            cameras=cameras,
            fps=30,
            kind="leader",
        )

        recorder = LeRobotRecorder(
            task_name=args_cli.task_name,
            repo_id=args_cli.repo_id,
            dataset_root=args_cli.repo_root,
            fps=30,
            device=env.unwrapped.device,
            cameras=cameras,
            save_mp4=args_cli.save_mp4,
            depth=args_cli.depth,
            instance_id_seg=args_cli.instance_id_seg,
        )
        try:
            recorder.init_dataset()
        except ValueError:
            print(f"[ERROR]: Failed to initialize dataset. folder already exists")
            env.close()
            simulation_app.close()

    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            deltas = keyboard_control.get_joint_deltas(args_cli.joint_step)
            deltas = torch.tensor(deltas, device=targets.device, dtype=targets.dtype)
            targets = torch.clamp(targets + deltas, joint_limits[:, 0], joint_limits[:, 1])

            # single-arm jog target is broadcast to every parallel env, since
            # there's only one keyboard driving them
            actions[:, :] = targets

            obs, _, _, _, _ = env.step(actions)

            if action_log_mode:
                if keyboard_control.recording:
                    action_log_buffer.append(targets.detach().cpu().numpy().copy())
                elif was_recording:
                    # 'recording' just flipped off (via 'S' or 'R') -> flush this episode
                    if action_log_buffer:
                        action_log_episode_index += 1
                        log_path = os.path.join(
                            args_cli.action_log, f"episode_{action_log_episode_index:03d}.npz"
                        )
                        np.savez(
                            log_path,
                            actions=np.stack(action_log_buffer).astype(np.float32),
                            joint_names=np.array(JointJogKeyboardControl.JOINT_ORDER),
                        )
                        print(f"[INFO]: Saved action log ({len(action_log_buffer)} steps) -> {log_path}")
                    action_log_buffer = []
                was_recording = keyboard_control.recording

            if keyboard_control.reset_world:
                keyboard_control.reset_world = False
                env.reset()
                targets = default_targets.clone()
                continue

            if recording_mode and keyboard_control.recording:
                visual_obs = obs.get("visual", None)
                if visual_obs is None:
                    print(
                        "[WARNING]: No 'visual' observation group - recording requires a task with cameras"
                    )
                    keyboard_control.recording = False
                    continue
                # Extract joint positions from policy observation dict
                joint_pos_obs = obs["policy"]["joint_pos_obs"][0]
                visual_obs = obs["visual"]
                real_action = robot_iface.get_raw_actions_from_radians(targets)
                real_obs, visual_buffers, depth_buffers, instance_id_seg_buffers = (
                    robot_iface.sim_to_real_dataset_processor(joint_pos_obs, visual_obs)
                )
                recorder.push_frame_to_buffer(
                    real_action,
                    real_obs,
                    visual_buffers,
                    depth_buffers,
                    instance_id_seg_buffers,
                )

    env.close()


if __name__ == "__main__":

    main()

    while True:
        simulation_app.update()

    simulation_app.close()
