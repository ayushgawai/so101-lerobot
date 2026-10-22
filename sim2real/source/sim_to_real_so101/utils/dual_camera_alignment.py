"""Initial tabletop registration from simultaneous base/object ArUco detections.

Transforms use ``destination_from_source`` and meters. No camera extrinsics
are cached: the wrist may move BETWEEN images, but must be still during an
exposure. Objects and the measured base marker must stay fixed for the run.
Only OpenCV/NumPy and Isaac-free utilities are imported.
"""

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from .marker_tracking import detect_markers


CAMERAS = ("fixed", "wrist")
OBJECTS = ("cube", "bowl")
REST_Z = {"cube": 0.0575, "bowl": 0.03}


class AlignmentError(ValueError):
    """Invalid calibration/configuration or insufficient/inconsistent evidence."""


class ObservationRejected(AlignmentError):
    """A frame cannot supply an unambiguous, physically valid observation."""


def rigid_transform(value, name="transform"):
    """Validate a finite, proper SE(3) matrix (no scale or reflections)."""
    try:
        matrix = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise AlignmentError(f"{name} must be a numeric 4x4 rigid transform") from exc
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise AlignmentError(f"{name} must be a finite 4x4 rigid transform")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0)):
        raise AlignmentError(f"{name} must have an orthonormal proper rotation and homogeneous last row")
    return matrix.copy()


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise AlignmentError(f"{name} must be a finite positive number")
    return float(value)


def _read_json(path):
    try:
        with Path(path).open(encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, ValueError) as exc:
        raise AlignmentError(f"Cannot read JSON {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise AlignmentError(f"{path}: expected a JSON object")
    return data


@dataclass(frozen=True)
class Intrinsics:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    frame_size: tuple[int, int]

    @classmethod
    def load(cls, path):
        """Read the existing calibrate_camera_intrinsics.py JSON format."""
        data = _read_json(path)
        try:
            k = np.asarray(data["camera_matrix"], dtype=float)
            d = np.asarray(data["dist_coeffs"], dtype=float)
            size = data["frame_size"]
        except (KeyError, TypeError, ValueError) as exc:
            raise AlignmentError(f"{path}: intrinsics require camera_matrix, dist_coeffs, frame_size") from exc
        if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1], atol=1e-8, rtol=0)
                or not np.allclose([k[0, 1], k[1, 0]], 0, atol=1e-8, rtol=0)):
            raise AlignmentError(f"{path}: invalid OpenCV camera_matrix")
        if (d.ndim not in (1, 2) or (d.ndim == 2 and 1 not in d.shape)
                or d.size not in (4, 5, 8, 12, 14) or not np.isfinite(d).all()):
            raise AlignmentError(f"{path}: invalid OpenCV dist_coeffs")
        if (not isinstance(size, list) or len(size) != 2
                or any(type(n) is not int or n <= 0 for n in size)):
            raise AlignmentError(f"{path}: frame_size must be [width, height] in calibrated pixels")
        return cls(k, d.reshape(-1), tuple(size))

    def validate_frame(self, frame):
        if frame is None or frame.ndim not in (2, 3) or (frame.shape[1], frame.shape[0]) != self.frame_size:
            raise AlignmentError(f"Image resolution must match intrinsics {self.frame_size}; do not resize frames")


@dataclass(frozen=True)
class Marker:
    marker_id: int
    size_m: float
    transform: np.ndarray


@dataclass(frozen=True)
class Camera:
    index: int
    intrinsics_path: Path
    intrinsics: Intrinsics


@dataclass(frozen=True)
class Thresholds:
    min_samples_per_camera: int = 5
    max_reprojection_rms_px: float = 2.0
    max_scatter_m: float = 0.005
    max_angular_scatter_deg: float = 5.0
    max_camera_disagreement_m: float = 0.01
    max_camera_disagreement_deg: float = 5.0
    max_rest_height_error_m: float = 0.01
    max_tilt_deg: float = 10.0
    ambiguity_error_margin_px: float = 0.25
    ambiguity_translation_m: float = 0.005
    ambiguity_angle_deg: float = 5.0


@dataclass(frozen=True)
class AlignmentConfig:
    cameras: dict[str, Camera]
    markers: dict[str, Marker]
    thresholds: Thresholds
    source: str


def load_config(path):
    """Load measured marker transforms and separately calibrated intrinsics.

    Relative intrinsics paths are resolved against the configuration file.
    Every marker requires measured=true: sizes and mounting transforms in the
    example are proposals, not physical measurements.
    """
    path = Path(path).resolve()
    data = _read_json(path)
    try:
        if type(data["version"]) is not int or data["version"] != 1:
            raise AlignmentError("Alignment configuration version must be 1")
        for group in ("markers", "cameras"):
            if not isinstance(data[group], dict):
                raise AlignmentError(f"{group} must be a JSON object")
        markers = {}
        for name, marker_id in (("base", 2), ("cube", 0), ("bowl", 1)):
            entry = data["markers"][name]
            if not isinstance(entry, dict):
                raise AlignmentError(f"markers.{name} must be a JSON object")
            if entry.get("measured") is not True:
                raise AlignmentError(f"markers.{name}.measured must be true only AFTER measuring size and mounting transform")
            if type(entry["id"]) is not int or entry["id"] != marker_id:
                raise AlignmentError(f"markers.{name}.id must be {marker_id} (DICT_4X4_50)")
            field = "base_from_marker" if name == "base" else "marker_from_object"
            markers[name] = Marker(marker_id, _positive(entry["size_m"], f"markers.{name}.size_m"),
                                   rigid_transform(entry[field], f"markers.{name}.{field}"))
        values = data.get("thresholds", {})
        if not isinstance(values, dict) or set(values) - set(Thresholds.__dataclass_fields__):
            raise AlignmentError("Unknown or invalid thresholds; check example field names")
        thresholds = Thresholds(**values)
        for name in Thresholds.__dataclass_fields__:
            _positive(getattr(thresholds, name), f"thresholds.{name}")
        if type(thresholds.min_samples_per_camera) is not int or thresholds.min_samples_per_camera < 5:
            raise AlignmentError("min_samples_per_camera must be an integer >= 5")
        if thresholds.max_tilt_deg > 10 or thresholds.max_reprojection_rms_px > 2:
            raise AlignmentError("max_tilt_deg must be <=10 and max_reprojection_rms_px <=2")
        cameras = {}
        for name, index in (("fixed", 1), ("wrist", 0)):
            entry = data["cameras"][name]
            if not isinstance(entry, dict):
                raise AlignmentError(f"cameras.{name} must be a JSON object")
            if type(entry["index"]) is not int or entry["index"] != index:
                raise AlignmentError(f"cameras.{name}.index must be {index}")
            if not isinstance(entry["intrinsics_path"], str) or not entry["intrinsics_path"].strip():
                raise AlignmentError(f"cameras.{name}.intrinsics_path must name a calibration JSON file")
            intrinsics_path = (path.parent / entry["intrinsics_path"]).resolve()
            cameras[name] = Camera(index, intrinsics_path, Intrinsics.load(intrinsics_path))
        if cameras["fixed"].intrinsics_path == cameras["wrist"].intrinsics_path:
            raise AlignmentError("Each camera needs its own separately calibrated intrinsics file")
    except (KeyError, TypeError) as exc:
        raise AlignmentError(f"Invalid alignment configuration {path}: {exc}") from exc
    return AlignmentConfig(cameras, markers, thresholds, str(path))


def marker_points(size_m):
    """IPPE_SQUARE order matches marker_tracking.detect_markers corner order."""
    half = size_m / 2
    return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=float)


@dataclass(frozen=True)
class PoseCandidate:
    camera_from_marker: np.ndarray
    rms_px: float


def marker_pose_candidates(corners, size_m, intrinsics, max_rms_px=2.0):
    """Keep only visible, positive-depth, low-error IPPE square solutions."""
    corners = np.asarray(corners, dtype=float)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ObservationRejected("Invalid marker image corners")
    points = marker_points(size_m)
    try:
        result = cv2.solvePnPGeneric(points, corners, intrinsics.camera_matrix,
                                     intrinsics.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error as exc:
        raise ObservationRejected(f"IPPE failed: {exc}") from exc
    candidates = []
    if result[0]:
        for rvec, tvec in zip(result[1], result[2]):
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            rotation = cv2.Rodrigues(rvec)[0]
            translation = tvec.reshape(3)
            depths = (points @ rotation.T + translation)[:, 2]
            # The printed front normal (+Z) must point toward the camera.
            if np.any(depths <= 0) or np.dot(rotation[:, 2], translation) >= 0:
                continue
            projected = cv2.projectPoints(points, rvec, tvec, intrinsics.camera_matrix,
                                           intrinsics.dist_coeffs)[0].reshape(4, 2)
            rms = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1))))
            if not np.isfinite(rms) or rms > max_rms_px:
                continue
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, translation
            candidates.append(PoseCandidate(rigid_transform(transform), rms))
    if not candidates:
        raise ObservationRejected("No front-facing positive-depth IPPE solution within reprojection RMS limit")
    return sorted(candidates, key=lambda candidate: candidate.rms_px)


def rotation_distance_deg(first, second):
    cosine = (np.trace(first.T @ second) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))


def _tabletop_check(transform, name, thresholds):
    tilt = float(np.degrees(np.arccos(np.clip(transform[2, 2], -1, 1))))
    if tilt > thresholds.max_tilt_deg:
        raise ObservationRejected(f"{name} tilt {tilt:.2f} deg exceeds tabletop limit")
    error = abs(transform[2, 3] - REST_Z[name])
    if error > thresholds.max_rest_height_error_m:
        raise ObservationRejected(f"{name} root height {transform[2, 3]:.4f} m inconsistent with {REST_Z[name]:.4f} m")


def select_object_pose(base_candidates, object_candidates, base_marker, object_marker, name, thresholds):
    """Resolve the pair jointly using the known base mount and tabletop model.

    A plausible runner-up within ambiguity_error_margin_px must agree in pose;
    contradictory near-ties are rejected, never averaged. The camera must be
    above the table with optical +Z having a downward component in base axes.
    """
    feasible = []
    for base in base_candidates:
        camera_from_base = rigid_transform(base.camera_from_marker @ np.linalg.inv(base_marker.transform))
        base_from_camera = np.linalg.inv(camera_from_base)
        if base_from_camera[2, 3] <= 0.03 or camera_from_base[2, 2] >= 0:
            continue
        for obj in object_candidates:
            pose = rigid_transform(base_from_camera @ obj.camera_from_marker @ object_marker.transform)
            try:
                _tabletop_check(pose, name, thresholds)
            except ObservationRejected:
                continue
            score = float(np.sqrt((base.rms_px ** 2 + obj.rms_px ** 2) / 2))
            feasible.append((score, pose))
    if not feasible:
        raise ObservationRejected(f"{name}: no candidate satisfies base +Z-up, camera-above-table, tilt and resting height")
    feasible.sort(key=lambda item: item[0])
    score, best = feasible[0]
    for other_score, other in feasible[1:]:
        if other_score - score > thresholds.ambiguity_error_margin_px:
            continue
        if (np.linalg.norm(best[:3, 3] - other[:3, 3]) > thresholds.ambiguity_translation_m
                or rotation_distance_deg(best[:3, :3], other[:3, :3]) > thresholds.ambiguity_angle_deg):
            raise ObservationRejected(f"{name}: ambiguous IPPE solutions; change camera view or improve marker visibility")
    return best, score


def mean_pose(poses):
    """Translation mean and proper-rotation SVD mean, including yaw wraparound."""
    matrices = [rigid_transform(pose) for pose in poses]
    if not matrices:
        raise AlignmentError("Cannot average zero observations")
    result = np.eye(4)
    result[:3, 3] = np.mean([pose[:3, 3] for pose in matrices], axis=0)
    u, singular_values, vt = np.linalg.svd(np.mean([pose[:3, :3] for pose in matrices], axis=0))
    if min(singular_values) < 1e-6:
        raise AlignmentError("Contradictory rotations have no stable mean")
    result[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return rigid_transform(result)


class WorkspaceAlignment:
    """Collect evidence via observe(camera_name, frame), then build_layout().

    Missing markers are reported in observe's diagnostics. Resolution/config
    failures raise AlignmentError. No cached base detections or extrinsics are
    used, and each frame contributes at most one sample per visible object.
    """

    def __init__(self, config):
        self.config = config
        self.samples = {camera: {name: [] for name in OBJECTS} for camera in CAMERAS}
        self.errors = {camera: {name: [] for name in OBJECTS} for camera in CAMERAS}

    @property
    def complete(self):
        return all(len(self.samples[camera][name]) >= self.config.thresholds.min_samples_per_camera
                   for camera in CAMERAS for name in OBJECTS)

    def progress(self):
        return {camera: {name: len(samples) for name, samples in objects.items()}
                for camera, objects in self.samples.items()}

    def observe(self, camera_name, frame):
        if camera_name not in CAMERAS:
            raise AlignmentError(f"Unknown camera {camera_name}")
        intrinsics = self.config.cameras[camera_name].intrinsics
        intrinsics.validate_frame(frame)
        corners = detect_markers(frame)
        # Integer-pixel ArUco corners bias depth for small printed markers.
        # Refine on this exposure without changing marker_tracking's detector
        # or its behavior for existing callers elsewhere in the project.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        for marker_id, points in corners.items():
            if (np.min(points) >= 4 and np.max(points[:, 0]) < gray.shape[1] - 4
                    and np.max(points[:, 1]) < gray.shape[0] - 4):
                refined = cv2.cornerSubPix(gray, np.asarray(points, dtype=np.float32).reshape(4, 1, 2).copy(),
                                          (3, 3), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                corners[marker_id] = refined.reshape(4, 2)
        base = self.config.markers["base"]
        if base.marker_id not in corners:
            return {"base": "Base marker 2 missing in this image; no samples accepted"}
        thresholds = self.config.thresholds
        try:
            base_candidates = marker_pose_candidates(corners[base.marker_id], base.size_m, intrinsics,
                                                      thresholds.max_reprojection_rms_px)
        except ObservationRejected as exc:
            return {"base": str(exc)}
        diagnostics = {}
        for name in OBJECTS:
            if len(self.samples[camera_name][name]) >= thresholds.min_samples_per_camera:
                continue
            marker = self.config.markers[name]
            if marker.marker_id not in corners:
                diagnostics[name] = f"Object marker {marker.marker_id} missing in this image"
                continue
            try:
                candidates = marker_pose_candidates(corners[marker.marker_id], marker.size_m, intrinsics,
                                                     thresholds.max_reprojection_rms_px)
                pose, rms = select_object_pose(base_candidates, candidates, base, marker, name, thresholds)
                self.samples[camera_name][name].append(pose)
                self.errors[camera_name][name].append(rms)
                diagnostics[name] = "accepted"
            except ObservationRejected as exc:
                diagnostics[name] = str(exc)
        return diagnostics

    def build_layout(self, source_mode="live"):
        """Validate all samples before returning a FixedWorkspaceLayout v1 dict.

        Each camera gets equal weight. Scatter is maximum distance/angle from
        that camera's mean, not standard error (which could hide bad samples).
        Cube Z is the loader's rest_on_table sentinel; measured Z is metadata.
        """
        if not self.complete:
            raise AlignmentError(f"Need >= {self.config.thresholds.min_samples_per_camera} samples for EACH object from BOTH cameras: {self.progress()}")
        thresholds = self.config.thresholds
        metadata = {}
        fused = {}
        for name in OBJECTS:
            per_camera = {}
            means = []
            for camera in CAMERAS:
                poses = self.samples[camera][name]
                for pose in poses:
                    _tabletop_check(rigid_transform(pose), name, thresholds)
                mean = mean_pose(poses)
                scatter = max(float(np.linalg.norm(pose[:3, 3] - mean[:3, 3])) for pose in poses)
                angular_scatter = max(rotation_distance_deg(mean[:3, :3], pose[:3, :3]) for pose in poses)
                if scatter > thresholds.max_scatter_m or angular_scatter > thresholds.max_angular_scatter_deg:
                    raise AlignmentError(f"{camera}/{name}: scatter {scatter:.4f} m, {angular_scatter:.2f} deg exceeds limits")
                means.append(mean)
                per_camera[camera] = {"samples": len(poses), "max_scatter_m": scatter,
                                      "max_angular_scatter_deg": angular_scatter,
                                      "mean_base_from_object": mean.tolist(),
                                      "max_pair_reprojection_rms_px": max(self.errors[camera][name], default=0.0)}
            distance = float(np.linalg.norm(means[0][:3, 3] - means[1][:3, 3]))
            angle = rotation_distance_deg(means[0][:3, :3], means[1][:3, :3])
            if distance > thresholds.max_camera_disagreement_m or angle > thresholds.max_camera_disagreement_deg:
                raise AlignmentError(f"{name}: cameras disagree by {distance:.4f} m, {angle:.2f} deg")
            fused[name] = mean_pose(means)
            _tabletop_check(fused[name], name, thresholds)
            metadata[name] = {"per_camera": per_camera, "camera_disagreement_m": distance,
                              "camera_disagreement_deg": angle,
                              "measured_base_from_object": fused[name].tolist()}
        def object_fields(name):
            pose = fused[name]
            # Snap validated tabletop heights to the scene's resting model.
            return {"xyz_base_m": [float(pose[0, 3]), float(pose[1, 3]), 0.0 if name == "cube" else REST_Z[name]],
                    "yaw_deg": float(np.degrees(np.arctan2(pose[1, 0], pose[0, 0])))}
        layout = {
            "version": 1, "frame": "so101_base",
            "robot_world": {"xyz_m": [0.0, 0.3, 0.72], "yaw_deg": 0.0},
            "table": {"surface_z_world_m": 0.75},
            "cube": {**object_fields("cube"), "size_m": [0.055] * 3, "mass_kg": 0.06, "rest_on_table": True},
            "bowl": object_fields("bowl"),
            "alignment": {"method": "same_image_base_reference_ippe_square", "dictionary": "DICT_4X4_50",
                          "target_scene": "codex-pick-place-table.usd",
                          "source_mode": source_mode, "config_path": self.config.source,
                          "thresholds": vars(thresholds), "objects": metadata,
                          "cameras": {name: {"index": camera.index, "intrinsics_path": str(camera.intrinsics_path),
                                              "frame_size": list(camera.intrinsics.frame_size)}
                                      for name, camera in self.config.cameras.items()},
                          "markers": {name: {"id": marker.marker_id, "size_m": marker.size_m,
                                              "base_from_marker" if name == "base" else "marker_from_object": marker.transform.tolist()}
                                      for name, marker in self.config.markers.items()},
                          "height_policy": "Validate measured roots; cube z=0 is rest_on_table sentinel; bowl z=0.03"},
        }
        from .fixed_workspace import parse_layout_dict
        parse_layout_dict(layout, source="<dual-camera-registration>")
        return layout
