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
"""ArUco marker detection and world-frame pose recovery, deliberately free of
any isaaclab/isaacsim import so it stays unit-testable without a booted Kit
process (matches utils/geometry.py's convention) -- see
docs/object-pose-mirroring-plan.md Phase 1.

Quaternions are (w, x, y, z), matching utils/geometry.py's convention.

Uses cv2.aruco.ArucoDetector + manual solvePnP, not
cv2.aruco.estimatePoseSingleMarkers -- that convenience function does not
exist in this OpenCV build (4.13.0, verified 2026-08-18 against both
C:\\Isaac-Sim\\python.bat and so101-lerobot's venv), so this is the
maintained replacement, not a stylistic choice.
"""
import json
from dataclasses import dataclass

import cv2
import numpy as np

_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_DETECTOR = cv2.aruco.ArucoDetector(_DICTIONARY, cv2.aruco.DetectorParameters())


@dataclass
class CameraCalibration:
    """One physical camera's intrinsics plus its one-time extrinsic
    calibration against the same world frame real-to-sim.usd authors (see
    docs/object-pose-mirroring-plan.md Phase 0).
    """

    camera_matrix: np.ndarray
    """(3, 3) intrinsic matrix, from cv2.calibrateCamera."""

    dist_coeffs: np.ndarray
    """(N,) distortion coefficients, from cv2.calibrateCamera."""

    world_from_camera: np.ndarray
    """(4, 4) homogeneous transform: point_world = world_from_camera @ point_camera.
    From a one-time solvePnP against a marker at a precisely-measured world
    position (Phase 0) -- invalid if the camera is ever bumped/remounted."""


def load_camera_calibration(intrinsics_path: str, extrinsics_path: str) -> CameraCalibration:
    """Load a CameraCalibration from the two JSON files Phase 0's
    calibration step produces (calibration/camera/top_camera_intrinsics.json,
    top_camera_extrinsics.json)."""
    with open(intrinsics_path) as f:
        intrinsics = json.load(f)
    with open(extrinsics_path) as f:
        extrinsics = json.load(f)
    return CameraCalibration(
        camera_matrix=np.array(intrinsics["camera_matrix"], dtype=np.float64),
        dist_coeffs=np.array(intrinsics["dist_coeffs"], dtype=np.float64),
        world_from_camera=np.array(extrinsics["world_from_camera"], dtype=np.float64),
    )


def detect_markers(frame: np.ndarray) -> dict[int, np.ndarray]:
    """Detect ArUco (DICT_4X4_50) markers in a BGR frame.

    Returns a dict mapping marker id -> its 4 corner pixel coords, shape
    (4, 2), in cv2.aruco's own order (top-left, top-right, bottom-right,
    bottom-left). Empty dict if none detected -- callers should hold the
    last known pose on a miss, not treat it as an error: occlusion of the
    marker by the gripper mid-grasp is the expected, common case, not a
    fault condition (see Phase 2's marker-not-visible fallback).
    """
    corners, ids, _ = _DETECTOR.detectMarkers(frame)
    if ids is None:
        return {}
    return {int(marker_id[0]): marker_corners[0] for marker_id, marker_corners in zip(ids, corners)}


def solve_marker_pose_camera_frame(
    corners: np.ndarray, marker_size_m: float, calib: CameraCalibration
) -> tuple[np.ndarray, np.ndarray]:
    """Camera-frame (rvec, tvec) of one marker via solvePnP.

    Args:
        corners: (4, 2) pixel coords from detect_markers, same corner order.
        marker_size_m: The printed marker's physical side length, in
            meters -- get this wrong and every downstream pose is biased by
            the same ratio (see Phase 0's calibration note).
        calib: This camera's intrinsics.
    """
    half = marker_size_m / 2.0
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    ok, rvec, tvec = cv2.solvePnP(
        object_points, corners.astype(np.float64), calib.camera_matrix, calib.dist_coeffs
    )
    if not ok:
        raise RuntimeError("solvePnP failed to converge for this marker observation")
    return rvec, tvec


def marker_pose_to_world(
    rvec: np.ndarray, tvec: np.ndarray, calib: CameraCalibration
) -> tuple[np.ndarray, np.ndarray]:
    """World-frame (position, quaternion wxyz) of a marker, composing its
    camera-frame solvePnP pose with the camera's Phase-0 extrinsic.

    Returns:
        position: (3,) world-frame position, meters.
        quat_wxyz: (4,) unit quaternion, (w, x, y, z), world-frame orientation.
    """
    rot_marker_in_camera, _ = cv2.Rodrigues(rvec)
    camera_from_marker = np.eye(4)
    camera_from_marker[:3, :3] = rot_marker_in_camera
    camera_from_marker[:3, 3] = tvec.reshape(3)

    world_from_marker = calib.world_from_camera @ camera_from_marker
    position = world_from_marker[:3, 3]
    quat_wxyz = _rotation_matrix_to_quat_wxyz(world_from_marker[:3, :3])
    return position, quat_wxyz


def quat_wxyz_to_rotation_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) -> (3, 3) rotation matrix. Inverse of
    _rotation_matrix_to_quat_wxyz, used by the Phase 0 extrinsic
    calibration script to build world_from_marker from a measured pose."""
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rotation_matrix_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    """(3, 3) rotation matrix -> unit quaternion, (w, x, y, z).

    Standard branch-selecting (Shepperd's method) conversion for numerical
    stability across the full rotation range -- the single sqrt(trace+1)
    formula alone loses precision (and can hit a near-zero denominator)
    near 180-degree rotations.
    """
    trace = np.trace(rot)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])
