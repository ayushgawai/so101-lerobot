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
"""Top camera extrinsic (camera -> world) calibration -- Phase 0 (part 2/2)
of docs/object-pose-mirroring-plan.md.

One-time step per camera mounting: place an ArUco marker at a
precisely-measured position in the same world frame real-to-sim.usd
already authors, then run this.

**Recommended placement: flat on TOP of the real cube, resting at its
normal spot** -- not flat on the bare table. This is deliberate, not
arbitrary: real-to-sim.usd authors AWSBuilderCube assuming a 5cm cube
(verified 2026-08-18 by inspecting the mesh extent directly), but the
*real* cube measures 5.7cm (also 2026-08-18, measured). Using the cube's
own authored CENTER position (0, 0.03, 0.7754) as a "place it flat here"
target is wrong on two counts: that's the cube's center, floating 2.5cm
above the table, not a height a flat marker can actually sit at; and even
"table height" alone (0.7504, from AWSCubePaper's authored position,
which sits flush with the cube's bottom face) would still be silently
wrong by using the sim's assumed 5cm instead of the real 5.7cm cube if you
then tried to add a height offset for stacking anything on it.

Placing the marker on top of the real cube sidesteps all of that: it
directly uses the real, measured object, at the exact spot it actually
rests, with no abstract "find table height in world coordinates" step.
DEFAULT_WORLD_POS_Z below is TABLE_SURFACE_Z (0.7504, from the sim's own
authored geometry) + REAL_CUBE_HEIGHT_M (0.057, the actual measurement) --
override --world_pos directly if your cube's measured height differs.

Requires top_camera_intrinsics.json to already exist (run
calibrate_camera_intrinsics.py first).

Console-only, no cv2.imshow -- see calibrate_camera_intrinsics.py's
module docstring for why.

Averages --num_samples detections' positions for robustness against single-
frame noise; reports the position spread across samples as a quality
check, but does NOT average the rotation (quaternion averaging has known
correctness pitfalls -- sign ambiguity, non-commutativity -- not worth the
complexity here). Instead uses the final sample's rotation once the spread
is small, which is a reasonable simplification given the calibration
marker is expected to sit flat and still throughout capture, not tumbling.

Usage:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\calibrate_camera_extrinsics.py \\
        --marker_id 0 --marker_size_m 0.03
"""
import argparse
import json
import os
import sys
import time

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

# From real-to-sim.usd's authored geometry directly (AWSCubePaper sits
# flush with AWSBuilderCube's bottom face -- see [[real_to_sim_aws_cube_bowl_task]]),
# NOT from the cube's own center position (0.7754), which is 2.5cm higher
# and assumes the sim's 5cm cube, not the real one.
_TABLE_SURFACE_Z = 0.7504
# Measured 2026-08-18. Override via --world_pos if your cube differs.
_REAL_CUBE_HEIGHT_M = 0.057
_DEFAULT_WORLD_POS_Z = _TABLE_SURFACE_Z + _REAL_CUBE_HEIGHT_M

parser = argparse.ArgumentParser(description="Top camera extrinsic calibration via a marker at a known world position.")
parser.add_argument("--camera_index", type=int, default=1, help="Top camera's OpenCV index.")
parser.add_argument("--marker_id", type=int, required=True, help="ArUco id of the calibration marker.")
parser.add_argument("--marker_size_m", type=float, required=True, help="Calibration marker's physical side length, meters.")
parser.add_argument(
    "--world_pos",
    type=float,
    nargs=3,
    default=(0.0, 0.03, _DEFAULT_WORLD_POS_Z),
    metavar=("X", "Y", "Z"),
    help="Measured world-frame position of the calibration marker's center. Default: table surface Z "
    "(from real-to-sim.usd's own geometry) plus the REAL cube's measured height (5.7cm) -- i.e. place "
    "the marker flat on top of the real cube, resting at its normal table position. Override the Z "
    "component if your cube's measured height differs from 5.7cm.",
)
parser.add_argument(
    "--world_quat_wxyz",
    type=float,
    nargs=4,
    default=(1.0, 0.0, 0.0, 0.0),
    metavar=("W", "X", "Y", "Z"),
    help="Measured world-frame orientation of the calibration marker, wxyz. Default: identity "
    "(marker flat, axis-aligned with the world frame).",
)
parser.add_argument("--num_samples", type=int, default=15, help="Detections to average before solving.")
parser.add_argument(
    "--intrinsics_path",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration", "camera", "top_camera_intrinsics.json"),
)
parser.add_argument(
    "--output_path",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration", "camera", "top_camera_extrinsics.json"),
)
args_cli = parser.parse_args()

if not os.path.isfile(args_cli.intrinsics_path):
    print(f"[ERROR]: Intrinsics file not found: {args_cli.intrinsics_path}", file=sys.stderr)
    print("[ERROR]: Run calibrate_camera_intrinsics.py first.", file=sys.stderr)
    sys.exit(1)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from sim_to_real_so101.utils.marker_tracking import (  # noqa: E402
    CameraCalibration,
    detect_markers,
    quat_wxyz_to_rotation_matrix,
    solve_marker_pose_camera_frame,
)


def main():
    with open(args_cli.intrinsics_path) as f:
        intrinsics = json.load(f)
    camera_matrix = np.array(intrinsics["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(intrinsics["dist_coeffs"], dtype=np.float64)
    # world_from_camera not known yet -- this dummy calibration object only
    # carries intrinsics, needed by solve_marker_pose_camera_frame.
    calib_intrinsics_only = CameraCalibration(
        camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, world_from_camera=np.eye(4)
    )

    world_from_marker = np.eye(4)
    world_from_marker[:3, :3] = quat_wxyz_to_rotation_matrix(np.array(args_cli.world_quat_wxyz))
    world_from_marker[:3, 3] = np.array(args_cli.world_pos)

    cap = cv2.VideoCapture(args_cli.camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open camera index={args_cli.camera_index}. Check the top camera is plugged in "
            "and not held open by another process (e.g. Windows Camera app)."
        )

    print(f"[INFO]: Looking for marker id={args_cli.marker_id} at world_pos={args_cli.world_pos}.")
    print(f"[INFO]: Need {args_cli.num_samples} detections.")

    world_from_camera_samples = []
    last_sample_time = 0.0
    try:
        while len(world_from_camera_samples) < args_cli.num_samples:
            ok, frame = cap.read()
            if not ok:
                print("[WARN]: Camera read failed this frame, skipping.")
                continue

            markers = detect_markers(frame)
            if args_cli.marker_id not in markers:
                continue

            now = time.perf_counter()
            if now - last_sample_time < 0.2:
                continue

            rvec, tvec = solve_marker_pose_camera_frame(
                markers[args_cli.marker_id], args_cli.marker_size_m, calib_intrinsics_only
            )
            rot_marker_in_camera, _ = cv2.Rodrigues(rvec)
            camera_from_marker = np.eye(4)
            camera_from_marker[:3, :3] = rot_marker_in_camera
            camera_from_marker[:3, 3] = tvec.reshape(3)

            world_from_camera_samples.append(world_from_marker @ np.linalg.inv(camera_from_marker))
            last_sample_time = now
            print(f"[INFO]: Captured sample {len(world_from_camera_samples)}/{args_cli.num_samples}.")
    except KeyboardInterrupt:
        print(
            f"\n[WARN]: Interrupted with {len(world_from_camera_samples)}/{args_cli.num_samples} samples -- "
            "not enough to solve, exiting."
        )
        cap.release()
        sys.exit(1)
    finally:
        cap.release()

    positions = np.array([m[:3, 3] for m in world_from_camera_samples])
    position_spread_m = positions.max(axis=0) - positions.min(axis=0)
    print(f"[INFO]: Position spread across samples (x, y, z): {position_spread_m} m.")
    if np.max(position_spread_m) > 0.01:
        print("[WARN]: Spread exceeds 1cm on some axis -- camera or marker may have moved during capture. "
              "Consider re-running before trusting this calibration.")

    world_from_camera = world_from_camera_samples[-1].copy()
    world_from_camera[:3, 3] = positions.mean(axis=0)

    os.makedirs(os.path.dirname(args_cli.output_path), exist_ok=True)
    with open(args_cli.output_path, "w") as f:
        json.dump(
            {
                "world_from_camera": world_from_camera.tolist(),
                "calibration_marker_id": args_cli.marker_id,
                "calibration_world_pos": list(args_cli.world_pos),
                "position_spread_m": position_spread_m.tolist(),
            },
            f,
            indent=2,
        )
    print(f"[INFO]: Wrote {args_cli.output_path}")


if __name__ == "__main__":
    main()
