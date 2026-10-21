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
"""Generate printable ArUco markers -- Phase 0, step 0 (before either
calibration script) of docs/object-pose-mirroring-plan.md.

Uses cv2.aruco.generateImageMarker, not the deprecated cv2.aruco.drawMarker
(which doesn't exist in this OpenCV build, 4.13.0, verified 2026-08-18 --
same class of removed-API issue as estimatePoseSingleMarkers elsewhere in
this plan).

Dictionary is DICT_4X4_50, hardcoded to match marker_tracking.py's own
_DICTIONARY exactly -- these must stay in sync, which is why this script
doesn't expose a --dictionary flag.

By default, generates id 0 (the cube's marker, matching
leader_arm_teleop_raw_isaacsim.py's --cube_marker_id default) and id 1
(for the bowl, Phase 3, not yet wired up). The SAME id-0 marker doubles as
the extrinsic-calibration marker in calibrate_camera_extrinsics.py's
default workflow -- place it where the cube normally rests, no separate
third marker needed.

Usage:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\generate_aruco_markers.py

Then print calibration/markers/aruco_id0.png at "actual size / 100%" (not
"fit to page") -- and regardless of what your printer actually produces,
MEASURE the printed black square with calipers/a ruler afterward and use
that exact number for --marker_size_m everywhere else in this plan.
Printer scaling is not trustworthy enough to skip this measurement.
"""
import argparse
import os

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Generate printable ArUco (DICT_4X4_50) marker images.")
parser.add_argument(
    "--marker_ids", type=int, nargs="+", default=[0, 1], help="Marker ids to generate. Default: 0 (cube), 1 (bowl)."
)
parser.add_argument("--pixels_per_marker", type=int, default=1000, help="Marker resolution, before the white margin.")
parser.add_argument(
    "--quiet_zone_fraction",
    type=float,
    default=0.2,
    help="White margin around the marker, as a fraction of its own size. ArUco detection is unreliable "
    "without a white border -- printed edges/shadows can eat into it otherwise, so don't set this to 0.",
)
parser.add_argument(
    "--target_size_cm",
    type=float,
    default=4.5,
    help="Physical marker size to print at (embedded as image DPI so 'print at 100%%/actual size' matches "
    "this) -- guidance only, printers often ignore embedded DPI or default to 'fit to page'. Default 4.5cm "
    "leaves a margin on the real cube's measured 5.7cm face (2026-08-18 measurement -- the sim's authored "
    "cube is a smaller, mismatched 5cm, see [[real_to_sim_aws_cube_bowl_task]]; don't use that for sizing "
    "against the real object). ALWAYS measure the actual printed marker afterward regardless.",
)
parser.add_argument(
    "--output_dir", type=str, default=os.path.join(_REPO_ROOT_DIR, "calibration", "markers")
)
args_cli = parser.parse_args()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def main():
    os.makedirs(args_cli.output_dir, exist_ok=True)
    dpi = args_cli.pixels_per_marker / (args_cli.target_size_cm / 2.54)
    margin_px = int(args_cli.pixels_per_marker * args_cli.quiet_zone_fraction)
    padded_size = args_cli.pixels_per_marker + 2 * margin_px

    for marker_id in args_cli.marker_ids:
        marker_img = cv2.aruco.generateImageMarker(_DICTIONARY, marker_id, args_cli.pixels_per_marker)
        padded = np.full((padded_size, padded_size), 255, dtype=np.uint8)
        padded[margin_px : margin_px + args_cli.pixels_per_marker, margin_px : margin_px + args_cli.pixels_per_marker] = (
            marker_img
        )

        output_path = os.path.join(args_cli.output_dir, f"aruco_id{marker_id}.png")
        Image.fromarray(padded).save(output_path, dpi=(dpi, dpi))
        print(f"[INFO]: Wrote {output_path} ({padded_size}x{padded_size}px, DPI={dpi:.1f})")

    print(f"\n[INFO]: Suggested print size: {args_cli.target_size_cm}cm per marker (the black/white square only, not the extra white margin).")
    print(
        "[INFO]: In your print dialog, disable 'fit to page'/'scale to fit' and print at 100%/actual size "
        "if your viewer supports it -- DPI is embedded to help with that."
    )
    print(
        "[INFO]: Regardless of print settings, MEASURE the printed marker's black/white square with a ruler "
        "or calipers afterward. Use that exact measurement, not --target_size_cm, for --marker_size_m in "
        "every other Phase 0/1/2 script -- printer scaling is not trustworthy enough to skip this."
    )


if __name__ == "__main__":
    main()
