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
"""Top camera intrinsic calibration -- Phase 0 (part 1/2) of
docs/object-pose-mirroring-plan.md.

Console-only, no cv2.imshow/namedWindow -- Isaac Sim's cv2 build has no
GUI backend at all (verified 2026-08-18: no WIN32UI/GTK in
cv2.getBuildInformation()), same class of headless-opencv issue hit
earlier getting so101-lerobot's eval camera preview working. Auto-captures
a sample whenever a checkerboard is detected and enough time has passed
since the last capture -- move/tilt the board between beeps rather than
holding it still, standard camera-calibration practice for a
well-conditioned solve.

Needs a printed checkerboard (any standard one; OpenCV's own docs have a
printable 9x6-inner-corner target) with a precisely measured square size.

Usage:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\calibrate_camera_intrinsics.py \\
        --square_size_m 0.025
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

parser = argparse.ArgumentParser(description="Top camera intrinsic calibration via a printed checkerboard.")
parser.add_argument("--camera_index", type=int, default=1, help="Top camera's OpenCV index.")
parser.add_argument("--cols", type=int, default=9, help="Inner corners per row of the printed checkerboard.")
parser.add_argument("--rows", type=int, default=6, help="Inner corners per column of the printed checkerboard.")
parser.add_argument(
    "--square_size_m", type=float, required=True, help="One checkerboard square's physical side length, meters."
)
parser.add_argument("--num_samples", type=int, default=20, help="Checkerboard detections to collect before solving.")
parser.add_argument(
    "--min_interval_s", type=float, default=1.0, help="Minimum time between accepted samples -- move the board between them."
)
parser.add_argument(
    "--output_path",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration", "camera", "top_camera_intrinsics.json"),
)
args_cli = parser.parse_args()

import cv2  # noqa: E402
import numpy as np  # noqa: E402


def main():
    board_size = (args_cli.cols, args_cli.rows)

    object_point_template = np.zeros((args_cli.cols * args_cli.rows, 3), dtype=np.float64)
    object_point_template[:, :2] = np.mgrid[0 : args_cli.cols, 0 : args_cli.rows].T.reshape(-1, 2)
    object_point_template *= args_cli.square_size_m

    cap = cv2.VideoCapture(args_cli.camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open camera index={args_cli.camera_index}. Check the top camera is plugged in "
            "and not held open by another process (e.g. Windows Camera app)."
        )

    object_points = []
    image_points = []
    frame_size = None
    last_sample_time = 0.0
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(f"[INFO]: Looking for a {args_cli.cols}x{args_cli.rows}-inner-corner checkerboard.")
    print(f"[INFO]: Need {args_cli.num_samples} samples, >= {args_cli.min_interval_s}s apart -- move/tilt the board between them.")

    try:
        while len(object_points) < args_cli.num_samples:
            ok, frame = cap.read()
            if not ok:
                print("[WARN]: Camera read failed this frame, skipping.")
                continue
            frame_size = (frame.shape[1], frame.shape[0])

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, board_size)
            if not found:
                continue

            now = time.perf_counter()
            if now - last_sample_time < args_cli.min_interval_s:
                continue

            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
            object_points.append(object_point_template.copy())
            image_points.append(corners_refined)
            last_sample_time = now
            print(f"[INFO]: Captured sample {len(object_points)}/{args_cli.num_samples}.")
    except KeyboardInterrupt:
        print(f"\n[WARN]: Interrupted with {len(object_points)}/{args_cli.num_samples} samples -- not enough to solve, exiting.")
        cap.release()
        sys.exit(1)
    finally:
        cap.release()

    print("[INFO]: Solving cv2.calibrateCamera...")
    reproj_error, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points, image_points, frame_size, None, None
    )
    print(f"[INFO]: Reprojection error: {reproj_error:.4f} px (want well under 1.0 px).")

    os.makedirs(os.path.dirname(args_cli.output_path), exist_ok=True)
    with open(args_cli.output_path, "w") as f:
        json.dump(
            {
                "camera_matrix": camera_matrix.tolist(),
                "dist_coeffs": dist_coeffs.flatten().tolist(),
                "reprojection_error_px": reproj_error,
                "frame_size": frame_size,
            },
            f,
            indent=2,
        )
    print(f"[INFO]: Wrote {args_cli.output_path}")


if __name__ == "__main__":
    main()
