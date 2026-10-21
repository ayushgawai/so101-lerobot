#!/usr/bin/env python
"""Drive the SO-101 follower to the demonstration start pose and hold it.

The ACT policy was trained on 50 demos that all start in a very tight pose
(elbow ~95 deg, folded up). Eval rollouts that start elsewhere are out of
distribution and the policy never gets on track. This positions the arm at the
demo start mean, then leaves torque ON so it holds while you launch lerobot-record.

Usage:
    .\.venv\Scripts\python.exe scripts\goto_start_pose.py
"""

import time
from pathlib import Path

import numpy as np

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# Demo start pose (mean over 50 training episodes), in degrees. gripper is 0-100 (closed ~2).
TARGET = {
    "shoulder_pan": 3.0,
    "shoulder_lift": -103.0,
    "elbow_flex": 95.0,
    "wrist_flex": 78.0,
    "wrist_roll": -89.0,
    "gripper": 2.0,
}

def main() -> None:
    cfg = SO101FollowerConfig(
        port="COM3",
        id="my_so_arm",
        calibration_dir=Path("./calibration/robots/so_follower"),
        cameras={},                          # no cameras needed to pose the arm
        use_degrees=True,
        disable_torque_on_disconnect=False,  # keep holding the pose after this script exits
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    try:
        current = robot.bus.sync_read("Present_Position")  # {motor: degrees}
        print("current pose:", {k: round(v, 1) for k, v in current.items()})
        print("target  pose:", TARGET)

        # Interpolate over ~2 s so the arm moves smoothly instead of snapping.
        steps = 60
        for t in np.linspace(0.0, 1.0, steps):
            action = {
                f"{m}.pos": float(current[m] + t * (TARGET[m] - current[m])) for m in TARGET
            }
            robot.send_action(action)
            time.sleep(2.0 / steps)

        time.sleep(0.5)
        final = robot.bus.sync_read("Present_Position")
        print("reached pose:", {k: round(v, 1) for k, v in final.items()})
        err = max(abs(final[m] - TARGET[m]) for m in TARGET)
        print(f"max joint error vs target: {err:.1f} deg")
        print("\nArm is holding the start pose (torque still ON).")
        print("Now launch the lerobot-record command in another terminal, then reset the")
        print("cube and press start. Re-run this script during each reset window.")
    finally:
        robot.disconnect()  # torque stays on (disable_torque_on_disconnect=False)

if __name__ == "__main__":
    main()
