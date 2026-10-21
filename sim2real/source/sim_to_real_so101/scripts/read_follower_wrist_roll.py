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
"""Read one joint angle (default: wrist_roll) off the REAL SO-101 follower.

No Isaac Sim, no Kit, no sim stage -- this is the standalone follower smoke
test from docs/dual-teleop-sim-and-follower-plan.md Phase 0, narrowed to a
single joint: connect, `get_observation()`, print, disconnect. Its whole
point is to isolate "is COM3 / the follower / its calibration OK" from "does
the sim integration work," so it deliberately shares nothing with
leader_arm_teleop_raw_isaacsim.py beyond LeRobotSO101Interface.

Reports the same reading in three spaces, because they answer different
questions:
  * normalized -- lerobot's own `<joint>.pos`. SO101Follower doesn't set
    use_degrees, so its motors are MotorNormMode.RANGE_M100_100: this is
    -100..100, NOT degrees. It's the number send_action() consumes, i.e.
    exactly what the teleop script mirrors to the arm.
  * sim degrees -- the same value pushed through
    LeRobotSO101Interface.get_mapped_actions_vectorized(), which is what the
    teleop script writes into the USD joint drives (wrist_roll maps onto
    SO101_USD_MAPPING's -160..160 deg).
  * raw ticks -- the STS3215's Present_Position read with normalize=False,
    i.e. pre-calibration counts. Only this one exposes encoder wraparound,
    which is why it's here (see --check_range below).

The wrist_roll entry in calibration/robots/so_follower/my_so_arm.json is
range_min=4095 range_max=8190, above a 12-bit STS3215's 0..4095 -- and a
`.drifted_8190` backup sits next to that file, so this exact joint has
needed re-verification before (the plan's Phase 0 flags it too). That is why
wrist_roll is this script's default joint, and why --check_range (on by
default) warns when the live ticks or the calibrated range fall outside
0..4095 instead of silently reporting a plausible-looking angle.

No motion is ever commanded -- no send_action(), no position target. But
*connecting* is not side-effect-free: SO101Follower.connect() calls
configure(), and its `bus.torque_disabled()` context manager re-enables
torque in a `finally`, so a freshly connected follower goes stiff and holds
its current pose. A read-only probe wants the opposite, so this script
disables torque again immediately after connecting -- back-drive the wrist
by hand while --watch runs and the value tracks. Pass --keep_torque if you'd
rather the arm hold position. Either way disconnect() ends with torque off
(SO101FollowerConfig.disable_torque_on_disconnect defaults True).

Needs a Python with both torch and the vendored `lerobot` fork. On this
machine that means Isaac Sim's python -- NOT the repo's usdenv, which has
neither -- even though no Isaac Sim/Kit API is used here:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\read_follower_wrist_roll.py --port COM3
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\read_follower_wrist_roll.py --port COM3 --watch
"""
import argparse
import os
import sys
import time

# Match the teleop scripts: make sim_to_real_so101 importable without an
# editable pip install into whatever Python this is run with.
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Read a real SO-101 follower joint angle (default: wrist_roll).")
parser.add_argument(
    "--port",
    type=str,
    default=os.getenv("FOLLOWER_PORT", "COM3"),
    help="Follower arm serial port (default: COM3, matching the dual-teleop plan).",
)
parser.add_argument(
    "--robot_id",
    type=str,
    default="my_so_arm",
    help="Follower calibration id (must match a file under calibration/robots/so_follower/).",
)
parser.add_argument(
    "--calibration_dir",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration"),
    help="Root calibration directory (expects <this>/robots/so_follower/<robot_id>.json). "
    "Sets HF_LEROBOT_CALIBRATION -- must be set before `lerobot` is imported.",
)
parser.add_argument(
    "--joint",
    type=str,
    default="wrist_roll",
    help="Which joint to read. Default wrist_roll (see the module docstring on why that one). One of: "
    "shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.",
)
parser.add_argument(
    "--watch",
    action="store_true",
    default=False,
    help="Keep reading until Ctrl+C instead of taking a single reading and exiting.",
)
parser.add_argument("--hz", type=float, default=10.0, help="--watch polling rate. Default 10.")
parser.add_argument(
    "--all_joints",
    action="store_true",
    default=False,
    help="Also print every other joint's normalized position alongside --joint.",
)
parser.add_argument(
    "--keep_torque",
    action="store_true",
    default=False,
    help="Leave torque enabled after connecting (connect() -> configure() turns it on). Default is to "
    "disable it again so the arm is limp and back-drivable for a read-only probe.",
)
parser.add_argument(
    "--check_range",
    action="store_true",
    default=True,
    help="Warn when live encoder ticks or the calibrated range fall outside an STS3215's 0..4095 "
    "(on by default -- see the module docstring on wrist_roll's 4095..8190 calibration).",
)
parser.add_argument(
    "--no_check_range", dest="check_range", action="store_false", help="Disable the --check_range warnings."
)
args_cli = parser.parse_args()

# Fail fast on a bad --robot_id/--calibration_dir before touching the serial
# port -- lerobot's own error for a missing calibration file surfaces much
# later, from deep inside connect()'s handshake.
_CALIBRATION_FILE = os.path.join(
    args_cli.calibration_dir, "robots", "so_follower", f"{args_cli.robot_id}.json"
)
if not os.path.isfile(_CALIBRATION_FILE):
    print(f"[ERROR]: Follower calibration file not found: {_CALIBRATION_FILE}", file=sys.stderr)
    print("[ERROR]: Check --robot_id / --calibration_dir.", file=sys.stderr)
    sys.exit(1)

# Must happen before any `lerobot` import (including transitively via
# sim_to_real_so101.utils.lerobot_interface) -- lerobot.utils.constants reads
# this at import time to compute HF_LEROBOT_CALIBRATION, which is what
# resolves calibration/robots/so_follower/<id>.json.
os.environ["HF_LEROBOT_CALIBRATION"] = args_cli.calibration_dir

import json  # noqa: E402

try:
    from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface  # noqa: E402
except ModuleNotFoundError as exc:
    # The repo's usdenv has neither torch nor lerobot, and `python` on this
    # machine resolves to it -- a bare ModuleNotFoundError here is almost
    # always "wrong interpreter," not "missing install."
    print(f"[ERROR]: {exc}", file=sys.stderr)
    print(f"[ERROR]: Running under {sys.executable}", file=sys.stderr)
    print(
        "[ERROR]: This script needs a Python with torch + the vendored `lerobot` fork -- use "
        "C:\\Isaac-Sim\\python.bat instead of a bare `python`.",
        file=sys.stderr,
    )
    sys.exit(1)

RAD_TO_DEG = 180.0 / 3.141592653589793
STS3215_TICKS_MAX = 4095  # 12-bit encoder, inclusive upper bound


def _read_raw_ticks(robot, joint: str):
    """Pre-calibration Present_Position, or None if unavailable. Wrapped in a
    try/except because this reaches past the Robot API into the motors bus --
    it's the only place encoder wraparound (e.g. wrist_roll's 8190) is
    actually visible, but it's also the most likely thing to break against a
    different lerobot revision, and a failure here shouldn't cost the
    normalized/degree readings that are this script's main job."""
    try:
        return robot.bus.sync_read("Present_Position", [joint], normalize=False)[joint]
    except Exception as exc:
        print(f"[WARN]: Could not read raw encoder ticks ({exc}) -- other values below are unaffected.")
        return None


def main():
    joint = args_cli.joint
    joint_key = f"{joint}.pos"
    if joint_key not in LeRobotSO101Interface.SO101_JOINT_ORDER:
        valid = ", ".join(j.split(".")[0] for j in LeRobotSO101Interface.SO101_JOINT_ORDER)
        print(f"[ERROR]: Unknown --joint {joint!r}. Valid joints: {valid}", file=sys.stderr)
        sys.exit(1)
    joint_index = LeRobotSO101Interface.SO101_JOINT_ORDER.index(joint_key)

    with open(_CALIBRATION_FILE, encoding="utf-8") as f:
        calibration = json.load(f)
    joint_calibration = calibration.get(joint, {})
    usd_range = LeRobotSO101Interface.SO101_USD_MAPPING[joint]

    print(f"[INFO]: calibration file = {_CALIBRATION_FILE}")
    print(
        f"[INFO]: {joint} calibration: id={joint_calibration.get('id')} "
        f"homing_offset={joint_calibration.get('homing_offset')} "
        f"range_min={joint_calibration.get('range_min')} range_max={joint_calibration.get('range_max')}"
    )
    print(f"[INFO]: {joint} sim/USD range: {usd_range['joint_min']}..{usd_range['joint_max']} deg")

    if args_cli.check_range:
        range_min = joint_calibration.get("range_min")
        range_max = joint_calibration.get("range_max")
        if range_min is not None and range_max is not None and (range_min < 0 or range_max > STS3215_TICKS_MAX):
            print(
                f"[WARN]: {joint}'s calibrated range {range_min}..{range_max} falls outside an STS3215's "
                f"0..{STS3215_TICKS_MAX} -- this calibration looks wrapped/drifted, so the angles below are "
                "suspect. Compare it against the .bak/.drifted_8190 siblings, or re-run lerobot's "
                "calibration for this arm."
            )

    # device="cpu" deliberately, unlike leader_arm_teleop_raw_isaacsim.py's
    # device="cuda" follower -- this script only maps a 6-element tensor, so a
    # GPU buys nothing, and requiring a CUDA torch build would narrow where a
    # Phase-0 smoke test can run.
    iface = LeRobotSO101Interface(
        device="cpu", port=args_cli.port, id=args_cli.robot_id, cameras={}, fps=30, kind="follower"
    )
    try:
        iface.init_device()
        iface.connect()
    except Exception as exc:
        print(
            f"[ERROR]: Failed to connect to the follower on port={args_cli.port} id={args_cli.robot_id}: {exc}",
            file=sys.stderr,
        )
        print(
            "[ERROR]: Check the arm is powered, plugged into the given --port, and not already held open by "
            "another process (e.g. a teleop run using --follower_port).",
            file=sys.stderr,
        )
        sys.exit(1)

    # connect() -> configure() leaves torque ON (its bus.torque_disabled()
    # context manager re-enables in a finally), which would fight anyone
    # back-driving the joint to check the reading tracks. Undo that unless
    # the caller explicitly wants the arm holding position.
    if not args_cli.keep_torque:
        try:
            iface.robot.bus.disable_torque()
            print("[INFO]: Torque disabled -- the arm is limp and back-drivable (pass --keep_torque to hold).")
        except Exception as exc:
            print(f"[WARN]: Could not disable torque ({exc}) -- the arm will hold its pose and resist back-driving.")

    period_s = 1.0 / args_cli.hz if args_cli.hz > 0 else 0.0
    try:
        while True:
            obs = iface.robot.get_observation()
            pos = {k: v for k, v in obs.items() if k.endswith(".pos")}
            if joint_key not in pos:
                print(f"[ERROR]: {joint_key} missing from the observation (got: {sorted(pos)})", file=sys.stderr)
                break

            normalized = pos[joint_key]
            # get_mapped_actions_vectorized works on the whole 6-joint vector
            # in SO101_JOINT_ORDER, so map all of them and index out the one
            # asked for -- same code path the teleop script drives the sim with.
            mapped_deg = iface.get_mapped_actions_vectorized(iface.get_raw_actions_tensor(pos)) * RAD_TO_DEG
            sim_deg = mapped_deg[joint_index].item()
            ticks = _read_raw_ticks(iface.robot, joint)

            ticks_str = "n/a" if ticks is None else f"{ticks}"
            print(f"{joint}: normalized={normalized:+8.3f}  sim_deg={sim_deg:+8.3f}  raw_ticks={ticks_str}")

            if args_cli.check_range and ticks is not None and not (0 <= ticks <= STS3215_TICKS_MAX):
                print(
                    f"[WARN]: raw ticks {ticks} outside 0..{STS3215_TICKS_MAX} -- the encoder has wrapped, so "
                    "the angle above is not trustworthy."
                )

            if args_cli.all_joints:
                others = "  ".join(
                    f"{k.split('.')[0]}={pos[k]:+7.2f}"
                    for k in LeRobotSO101Interface.SO101_JOINT_ORDER
                    if k in pos
                )
                print(f"  all: {others}")

            if not args_cli.watch:
                break
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\n[INFO]: Stopped.")
    finally:
        try:
            iface.robot.disconnect()
        except Exception as exc:
            print(f"[WARN]: Failed to cleanly disconnect the follower: {exc}")


if __name__ == "__main__":
    main()
