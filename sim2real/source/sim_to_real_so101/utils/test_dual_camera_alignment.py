"""Run with C:/Isaac-Sim/python.bat <this file>; no hardware or Isaac imports.

Synthetic projected corners test metric transforms; rendered ArUco sequences
exercise detection, IPPE, both camera streams, CLI and layout compatibility.
"""

import copy
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from sim_to_real_so101.utils import dual_camera_alignment as alignment
from sim_to_real_so101.utils.fixed_workspace import (
    load_layout as load_fixed_workspace_layout, parse_layout_dict, resolve_cube_pose,
)


SIM2REAL = Path(__file__).resolve().parents[3]
EXAMPLE = SIM2REAL / "calibration/workspaces/codex_camera_alignment.example.json"
CLI_PATH = SIM2REAL / "source/sim_to_real_so101/scripts/align_dual_camera_workspace.py"
spec = importlib.util.spec_from_file_location("alignment_cli", CLI_PATH)
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


def pose(xyz=(0, 0, 0), yaw=0, tilt=0):
    result = np.eye(4)
    yaw_r = cv2.Rodrigues(np.array([0.0, 0.0, np.radians(yaw)]))[0]
    tilt_r = cv2.Rodrigues(np.array([np.radians(tilt), 0.0, 0.0]))[0]
    result[:3, :3] = yaw_r @ tilt_r
    result[:3, 3] = xyz
    return result


def camera_pose(center):
    center = np.asarray(center)
    forward = np.array([0.03, -0.16, 0.03]) - center
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    rotation = np.stack([right, np.cross(forward, right), forward])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = -rotation @ center
    return result


class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.raw = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        self.intrinsics_data = {"camera_matrix": [[1200, 0, 640], [0, 1200, 480], [0, 0, 1]],
                                "dist_coeffs": [0, 0, 0, 0, 0], "frame_size": [1280, 960]}
        for name in alignment.CAMERAS:
            path = self.directory / f"{name}.json"
            path.write_text(json.dumps(self.intrinsics_data), encoding="utf-8")
            self.raw["cameras"][name]["intrinsics_path"] = path.name
        for marker in self.raw["markers"].values():
            marker["measured"] = True
        self.config_path = self.directory / "measured.json"
        self.save_config()
        self.config = alignment.load_config(self.config_path)
        self.objects = {"cube": pose([-0.055, -0.20, 0.0575], 24),
                        "bowl": pose([0.11, -0.21, 0.03], -32)}
        self.fixed = camera_pose([0.10, 0.13, 0.65])
        self.wrist = camera_pose([-0.14, 0.04, 0.48])
        self.blank = np.full((960, 1280, 3), 255, np.uint8)

    def save_config(self):
        self.config_path.write_text(json.dumps(self.raw), encoding="utf-8")

    def corners(self, camera_from_base, camera="fixed", config=None):
        config = config or self.config
        result = {}
        intrinsics = config.cameras[camera].intrinsics
        for name, marker in config.markers.items():
            base_from_marker = (marker.transform if name == "base"
                                else self.objects[name] @ np.linalg.inv(marker.transform))
            transform = camera_from_base @ base_from_marker
            result[marker.marker_id] = cv2.projectPoints(
                alignment.marker_points(marker.size_m), cv2.Rodrigues(transform[:3, :3])[0],
                transform[:3, 3], intrinsics.camera_matrix, intrinsics.dist_coeffs)[0].reshape(4, 2)
        return result

    def render(self, camera_from_base, camera):
        frame = self.blank.copy()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        source = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
        for marker_id, corners in self.corners(camera_from_base, camera).items():
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
            homography = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
            warped = cv2.warpPerspective(marker, homography, (1280, 960),
                                         flags=cv2.INTER_NEAREST, borderValue=255)
            frame = np.minimum(frame, cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR))
        return frame

    def filled(self):
        collector = alignment.WorkspaceAlignment(self.config)
        for camera in alignment.CAMERAS:
            for name in alignment.OBJECTS:
                collector.samples[camera][name] = [self.objects[name].copy() for _ in range(5)]
                collector.errors[camera][name] = [0.1] * 5
        return collector

    def test_example_requires_real_measurements(self):
        with self.assertRaisesRegex(alignment.AlignmentError, "measured"):
            alignment.load_config(EXAMPLE)
        self.raw["markers"]["bowl"]["measured"] = "true"
        self.save_config()
        with self.assertRaisesRegex(alignment.AlignmentError, "bowl.measured"):
            alignment.load_config(self.config_path)

    def test_missing_and_shared_intrinsics_fail(self):
        (self.directory / "fixed.json").unlink()
        with self.assertRaisesRegex(alignment.AlignmentError, "Cannot read JSON"):
            alignment.load_config(self.config_path)
        self.raw["cameras"]["fixed"]["intrinsics_path"] = "wrist.json"
        self.save_config()
        with self.assertRaisesRegex(alignment.AlignmentError, "separately calibrated"):
            alignment.load_config(self.config_path)

    def test_bad_intrinsics_and_resolution_fail(self):
        bad_cases = [dict(self.intrinsics_data, camera_matrix=[[float("nan")]*3]*3),
                     dict(self.intrinsics_data, dist_coeffs=[float("inf")]*5),
                     dict(self.intrinsics_data, frame_size=[0, 960]),
                     {key: value for key, value in self.intrinsics_data.items() if key != "frame_size"}]
        for data in bad_cases:
            with self.subTest(data=data):
                (self.directory / "bad.json").write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(alignment.AlignmentError):
                    alignment.Intrinsics.load(self.directory / "bad.json")
        with self.assertRaisesRegex(alignment.AlignmentError, "resolution"):
            alignment.WorkspaceAlignment(self.config).observe("fixed", np.zeros((480, 640, 3), np.uint8))

    def test_bad_transforms_and_thresholds_fail(self):
        for matrix in (np.diag([1, 1, -1, 1]), np.diag([2, 1, 1, 1]), np.full((4, 4), np.nan), np.eye(3)):
            with self.subTest(matrix=matrix), self.assertRaises(alignment.AlignmentError):
                alignment.rigid_transform(matrix)
        for field, value in (("min_samples_per_camera", 4), ("max_tilt_deg", 11),
                             ("max_reprojection_rms_px", 3), ("max_scatter_m", float("nan")),
                             ("max_camera_disagreement_m", -1), ("typo", 1)):
            with self.subTest(field=field):
                original = copy.deepcopy(self.raw["thresholds"])
                self.raw["thresholds"][field] = value
                self.save_config()
                with self.assertRaises(alignment.AlignmentError):
                    alignment.load_config(self.config_path)
                self.raw["thresholds"] = original

    def test_same_image_required_no_stale_base_pose(self):
        collector = alignment.WorkspaceAlignment(self.config)
        corners = self.corners(self.wrist, "wrist")
        with patch.object(alignment, "detect_markers", return_value={2: corners[2]}):
            collector.observe("wrist", self.blank)
        with patch.object(alignment, "detect_markers", return_value={0: corners[0], 1: corners[1]}):
            messages = collector.observe("wrist", self.blank)
        self.assertIn("Base marker 2 missing", messages["base"])
        self.assertEqual(collector.progress()["wrist"], {"cube": 0, "bowl": 0})

    def test_moving_wrist_and_full_mount_transforms_recover_metric_pose(self):
        # Exercise a nonzero reference yaw and object marker lateral/yaw offsets.
        markers = dict(self.config.markers)
        markers["base"] = replace(markers["base"], transform=pose([0, -0.1, 0.03], 35))
        markers["cube"] = replace(markers["cube"], transform=pose([0.002, -0.003, -0.0275], -16))
        config = replace(self.config, markers=markers)
        collector = alignment.WorkspaceAlignment(config)
        for sample in range(5):
            for camera in alignment.CAMERAS:
                transform = self.fixed if camera == "fixed" else camera_pose([-0.14 + sample*0.01, 0.04, 0.48 + sample*0.007])
                with patch.object(alignment, "detect_markers", return_value=self.corners(transform, camera, config)):
                    messages = collector.observe(camera, self.blank)
                self.assertEqual(messages, {"cube": "accepted", "bowl": "accepted"})
        self.assertTrue(collector.complete)
        layout = collector.build_layout("offline_frames")
        for name in alignment.OBJECTS:
            observed = layout["alignment"]["objects"][name]["measured_base_from_object"]
            # cornerSubPix uses float32 pixels; this bounds pose error to 1 um
            # in translation and about 1e-6 in rotation matrix components.
            np.testing.assert_allclose(observed, self.objects[name], atol=1e-6)

    def test_ippe_reprojection_threshold_and_depth_filter(self):
        intrinsics = self.config.cameras["fixed"].intrinsics
        corners = self.corners(self.fixed)[0]
        candidates = alignment.marker_pose_candidates(corners, 0.03, intrinsics)
        self.assertLess(candidates[0].rms_px, 1e-6)
        warped = corners.copy()
        warped[0] += [15, 10]
        with self.assertRaisesRegex(alignment.ObservationRejected, "reprojection"):
            alignment.marker_pose_candidates(warped, 0.03, intrinsics, 0.001)
        rvec = cv2.Rodrigues(self.fixed[:3, :3])[0]
        with patch.object(cv2, "solvePnPGeneric", return_value=(1, [rvec], [np.array([0., 0., -1.])])):
            with self.assertRaisesRegex(alignment.ObservationRejected, "positive-depth"):
                alignment.marker_pose_candidates(corners, 0.03, intrinsics)

    def test_contradictory_ippe_near_ties_are_rejected(self):
        base = self.config.markers["base"]
        cube = self.config.markers["cube"]
        base_candidate = alignment.PoseCandidate(self.fixed @ base.transform, 0.1)
        first = self.fixed @ self.objects["cube"] @ np.linalg.inv(cube.transform)
        alternate = self.objects["cube"].copy()
        alternate[0, 3] += 0.025
        second = self.fixed @ alternate @ np.linalg.inv(cube.transform)
        with self.assertRaisesRegex(alignment.ObservationRejected, "ambiguous IPPE"):
            alignment.select_object_pose([base_candidate], [alignment.PoseCandidate(first, 0.1),
                alignment.PoseCandidate(second, 0.12)], base, cube, "cube", self.config.thresholds)

    def test_camera_base_up_physical_constraint(self):
        base = self.config.markers["base"]
        cube = self.config.markers["cube"]
        below_table = pose([0, 0, 0.4])  # camera at base Z -0.4, optical Z upward
        with self.assertRaisesRegex(alignment.ObservationRejected, "base \\+Z-up"):
            alignment.select_object_pose(
                [alignment.PoseCandidate(below_table @ base.transform, 0)],
                [alignment.PoseCandidate(below_table @ self.objects["cube"] @ np.linalg.inv(cube.transform), 0)],
                base, cube, "cube", self.config.thresholds)

    def test_both_cameras_and_both_objects_need_five_samples(self):
        for camera in alignment.CAMERAS:
            for name in alignment.OBJECTS:
                with self.subTest(camera=camera, name=name):
                    collector = self.filled()
                    collector.samples[camera][name].pop()
                    self.assertFalse(collector.complete)
                    with self.assertRaisesRegex(alignment.AlignmentError, "BOTH cameras"):
                        collector.build_layout()

    def test_scatter_disagreement_tilt_and_height_rejections(self):
        for kind in ("scatter", "angular_scatter", "distance", "angle", "tilt", "height", "nan"):
            with self.subTest(kind=kind):
                collector = self.filled()
                poses = collector.samples["wrist"]["cube"]
                if kind == "scatter":
                    poses[0][0, 3] += 0.02
                elif kind == "angular_scatter":
                    poses[0][:3, :3] = pose(yaw=45)[:3, :3]
                else:
                    for matrix in poses:
                        if kind == "distance":
                            matrix[0, 3] += 0.012
                        elif kind == "angle":
                            matrix[:3, :3] = pose(yaw=31)[:3, :3]
                        elif kind == "tilt":
                            matrix[:3, :3] = pose(yaw=24, tilt=11)[:3, :3]
                        elif kind == "height":
                            matrix[2, 3] += 0.02
                        else:
                            matrix[0, 3] = np.nan
                with self.assertRaises(alignment.AlignmentError):
                    collector.build_layout()

    def test_yaw_wraparound_and_existing_layout_compatibility(self):
        collector = self.filled()
        for camera in alignment.CAMERAS:
            for index in range(5):
                collector.samples[camera]["cube"][index] = pose([-0.055, -0.2, 0.0575], 179 if index % 2 else -179)
        layout_dict = collector.build_layout()
        self.assertGreater(abs(layout_dict["cube"]["yaw_deg"]), 179)
        self.assertEqual(layout_dict["cube"]["xyz_base_m"][2], 0)
        self.assertEqual(layout_dict["bowl"]["xyz_base_m"][2], 0.03)
        layout = parse_layout_dict(layout_dict, "test")
        resolved = resolve_cube_pose(layout)
        self.assertAlmostEqual(resolved.xyz_world_m[2], 0.7775)

    def test_rendered_sequences_through_cli(self):
        entries = []
        for camera in alignment.CAMERAS:
            directory = self.directory / camera
            directory.mkdir()
            entries.append(f"{camera}={directory}")
            for index in range(6):
                transform = self.fixed if camera == "fixed" else camera_pose([-0.14 + index*0.005, 0.04, 0.48])
                self.assertTrue(cv2.imwrite(str(directory / f"{index:02}.png"), self.render(transform, camera)))
        output = self.directory / "registered.json"
        process = subprocess.run([sys.executable, str(CLI_PATH), "--config", str(self.config_path),
                                  "--output", str(output), "--timeout", "20", "--frames", *entries],
                                 capture_output=True, text=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        layout = json.loads(output.read_text(encoding="utf-8"))
        parsed = load_fixed_workspace_layout(str(output))
        self.assertTrue(parsed.cube.rest_on_table)
        self.assertEqual(parsed.cube.xyz_base_m[2], 0.0)
        self.assertAlmostEqual(resolve_cube_pose(parsed).xyz_world_m[2], 0.7775)
        self.assertEqual(layout["alignment"]["source_mode"], "offline_frames")
        for name in alignment.OBJECTS:
            np.testing.assert_allclose(layout[name]["xyz_base_m"][:2], self.objects[name][:2, 3], atol=0.004)
            self.assertLess(abs(layout[name]["yaw_deg"] - (24 if name == "cube" else -32)), 3)
            for camera in alignment.CAMERAS:
                self.assertGreaterEqual(layout["alignment"]["objects"][name]["per_camera"][camera]["samples"], 5)

    def test_cli_failure_never_creates_or_overwrites_output(self):
        output = self.directory / "must-not-exist.json"
        missing = self.directory / "missing.json"
        self.assertEqual(cli.main(["--config", str(missing), "--output", str(output)]), 1)
        self.assertFalse(output.exists())
        frame_path = self.directory / "blank.png"
        cv2.imwrite(str(frame_path), self.blank)
        args = ["--config", str(self.config_path), "--output", str(output), "--frames",
                f"fixed={frame_path}", f"wrist={frame_path}"]
        self.assertEqual(cli.main(args), 1)
        self.assertFalse(output.exists())
        output.write_text("existing output", encoding="utf-8")
        self.assertEqual(cli.main(args), 1)
        self.assertEqual(output.read_text(encoding="utf-8"), "existing output")

    def test_camera_open_failure_releases_other_camera_and_writes_nothing(self):
        output = self.directory / "missing-camera.json"
        with patch.object(cli.cv2, "VideoCapture") as capture:
            first, second = unittest.mock.MagicMock(), unittest.mock.MagicMock()
            first.isOpened.return_value = True
            second.isOpened.return_value = False
            capture.side_effect = [first, second]
            self.assertEqual(cli.main(["--config", str(self.config_path), "--output", str(output)]), 1)
            first.release.assert_called_once()
            second.release.assert_called_once()
        self.assertFalse(output.exists())

    def test_cli_rejects_invalid_timeout_and_frame_arguments(self):
        output = self.directory / "invalid.json"
        for value in ("nan", "inf", "0", "-1"):
            with self.subTest(timeout=value):
                self.assertEqual(cli.main(["--config", str(self.config_path), "--output", str(output),
                                           "--timeout", value]), 1)
                self.assertFalse(output.exists())
        for entries in (["fixed=file.png"], ["fixed=a", "fixed=b"], ["top=a", "wrist=b"]):
            with self.subTest(entries=entries), self.assertRaises(alignment.AlignmentError):
                cli.parse_frame_paths(entries)

    def test_live_timeout_releases_both_cameras_without_output(self):
        output = self.directory / "timeout.json"
        with patch.object(cli, "FrameSource") as source, patch.object(cli.time, "monotonic", side_effect=[0, 2]):
            self.assertEqual(cli.main(["--config", str(self.config_path), "--output", str(output),
                                       "--timeout", "1"]), 1)
            self.assertEqual(source.return_value.close.call_count, 2)
        self.assertFalse(output.exists())

    def test_offline_single_images_are_not_replayed_to_five(self):
        entries = []
        for camera, transform in (("fixed", self.fixed), ("wrist", self.wrist)):
            path = self.directory / f"one-{camera}.png"
            cv2.imwrite(str(path), self.render(transform, camera))
            entries.append(f"{camera}={path}")
        output = self.directory / "too-few.json"
        self.assertEqual(cli.main(["--config", str(self.config_path), "--output", str(output), "--frames", *entries]), 1)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
