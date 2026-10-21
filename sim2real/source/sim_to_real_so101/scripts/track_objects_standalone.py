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
"""Camera-only ArUco pose tracking, NO Isaac Sim -- Phase 1 of
docs/object-pose-mirroring-plan.md.

Isolates "does tracking work" from "does sim integration work," same
principle as the dual-teleop plan's Phase 0. Opens the top camera, prints
live cube/bowl world poses to the console. Requires Phase 0's calibration
files to already exist -- run the (separate, not-yet-written) camera
calibration step first if they don't.

Run with any Python that has opencv-contrib installed (this repo's own
`sim_to_real_so101` package doesn't need Isaac Sim's python for this
script specifically, unlike the teleop scripts -- but C:\\Isaac-Sim\\python.bat
works fine too, since it already has cv2.aruco, verified 2026-08-18):
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\track_objects_standalone.py --marker_size_m 0.03
"""
import argparse
import os
import sys
import time

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Standalone ArUco cube/bowl pose tracking (no Isaac Sim).")
parser.add_argument("--camera_index", type=int, default=1, help="Top camera's OpenCV index.")
parser.add_argument(
    "--marker_size_m",
    type=float,
    required=True,
    help="Printed marker's physical side length, meters. Must match exactly -- wrong by a few mm "
    "biases every downstream pose (see docs/object-pose-mirroring-plan.md Phase 0).",
)
parser.add_argument("--cube_marker_id", type=int, default=0, help="ArUco id printed on the cube's marker.")
parser.add_argument("--bowl_marker_id", type=int, default=1, help="ArUco id printed on the bowl's marker.")
parser.add_argument(
    "--calibration_dir",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration", "camera"),
    help="Directory with top_camera_intrinsics.json / top_camera_extrinsics.json (Phase 0 output).",
)
parser.add_argument("--print_interval_s", type=float, default=0.5, help="Console print rate (not the camera's own fps).")
args_cli = parser.parse_args()

_INTRINSICS_PATH = os.path.join(args_cli.calibration_dir, "top_camera_intrinsics.json")
_EXTRINSICS_PATH = os.path.join(args_cli.calibration_dir, "top_camera_extrinsics.json")
for _path in (_INTRINSICS_PATH, _EXTRINSICS_PATH):
    if not os.path.isfile(_path):
        print(f"[ERROR]: Calibration file not found: {_path}", file=sys.stderr)
        print(
            "[ERROR]: Run Phase 0's camera calibration first (see docs/object-pose-mirroring-plan.md) "
            "-- this script only reads calibration, it doesn't produce it.",
            file=sys.stderr,
        )
        sys.exit(1)

import cv2  # noqa: E402

from sim_to_real_so101.utils.marker_tracking import (  # noqa: E402
    detect_markers,
    load_camera_calibration,
    marker_pose_to_world,
    solve_marker_pose_camera_frame,
)


def main():
    calib = load_camera_calibration(_INTRINSICS_PATH, _EXTRINSICS_PATH)

    cap = cv2.VideoCapture(args_cli.camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open camera index={args_cli.camera_index}. Check the top camera is plugged in "
            "and not held open by another process (e.g. Windows Camera app -- a real cause seen on this "
            "machine, see docs/object-pose-mirroring-plan.md)."
        )

    tracked_ids = {args_cli.cube_marker_id: "cube", args_cli.bowl_marker_id: "bowl"}
    print(f"[INFO]: Tracking marker ids {tracked_ids} on camera index={args_cli.camera_index}.")
    print("[INFO]: Ctrl+C to stop.")

    last_print = time.perf_counter()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN]: Camera read failed this frame, skipping.")
                continue

            markers = detect_markers(frame)

            now = time.perf_counter()
            if now - last_print >= args_cli.print_interval_s:
                for marker_id, label in tracked_ids.items():
                    if marker_id not in markers:
                        print(f"[{label}] not detected this frame")
                        continue
                    rvec, tvec = solve_marker_pose_camera_frame(
                        markers[marker_id], args_cli.marker_size_m, calib
                    )
                    position, quat_wxyz = marker_pose_to_world(rvec, tvec, calib)
                    print(
                        f"[{label}] pos=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) "
                        f"quat_wxyz=({quat_wxyz[0]:.3f}, {quat_wxyz[1]:.3f}, {quat_wxyz[2]:.3f}, {quat_wxyz[3]:.3f})"
                    )
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


if __name__ == "__main__":
    main()
