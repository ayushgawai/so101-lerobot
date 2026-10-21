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
"""Fixed-layout real-to-sim workspace config: pure JSON/math, no isaacsim/omni
import (same "isaac-free, unit-testable" convention as utils/geometry.py) --
this module is safe to import and unit-test without a booted Kit process.

Replaces the old approach of hand-measured, unexplained world-frame constants
(``REAL_CUBE_POS``/``REAL_BOWL_POS``/``ROBOT_POS`` in
leader_arm_teleop_raw_isaacsim.py) with cube/bowl poses expressed relative to
the SO-101 base mounting point, converted to Isaac world poses at startup.
See docs/aws-cube-to-bowl-run-guide.md for the physical measurement
procedure.

Coordinate-frame convention (the one explicit transformation path this
module implements)::

    T_world_cube = T_world_base @ T_base_cube
    T_world_bowl = T_world_base @ T_base_bowl

- **Frame name**: ``"so101_base"``. Origin = the SO-101's mounting point on
  the table (where ``root_joint`` is pinned), i.e. ``robot_world.xyz_m`` in
  the layout JSON *is* ``T_world_base``'s translation.
- **Up axis**: +Z (matches ``real-to-sim.usd``'s stage ``upAxis`` and its
  Z-up table height convention, e.g. ``table.surface_z_world_m``).
- **+X / +Y**: same directions as the world/USD stage axes when
  ``robot_world.yaw_deg == 0`` (the base frame is not independently
  handed/mirrored from world -- only yawed about +Z).
- **Yaw convention**: right-handed rotation about +Z (CCW looking down the
  -Z axis from above), degrees in the JSON schema, everywhere else in this
  module's public API also degrees unless a name ends in ``_rad``. Roll and
  pitch are always zero -- this demo's fixture is a flat tabletop, planar
  yaw + XYZ is sufficient (see the module docstring in
  leader_arm_teleop_raw_isaacsim.py for why a full 6-DoF pose isn't needed).
- **Cube position = its geometric *center***, matching how
  ``AWSBuilderCube``'s ``xformOp:translate`` is already authored in
  real-to-sim.usd (confirmed by direct USD inspection: the cube's 8 mesh
  points are symmetric about its own local origin).
- **Bowl position = its table-contact point** (the bottom of its footprint),
  matching how ``PaperBowl``/``Bowl_Geo`` is already authored (its mesh
  extent starts at local z=0, i.e. the Xform's own translate already sits at
  the bowl's resting height) -- *not* its center, unlike the cube.
"""
import json
import math
import os
from dataclasses import dataclass

SUPPORTED_VERSION = 1
SUPPORTED_FRAME = "so101_base"

Vec3 = tuple[float, float, float]


class LayoutError(ValueError):
    """Raised for a malformed/invalid fixed-workspace layout config or an
    invalid CLI override of one -- always carries a human-readable message
    naming the offending field, so callers can print it directly and exit
    before Isaac Sim finishes booting."""


@dataclass(frozen=True)
class Pose2D:
    """A planar (yaw-only) pose: full XYZ translation, yaw about +Z. Roll/
    pitch are always zero (see module docstring)."""

    xyz_m: Vec3
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class CubeConfig:
    xyz_base_m: Vec3
    yaw_deg: float
    size_m: Vec3
    mass_kg: float
    rest_on_table: bool = False


@dataclass(frozen=True)
class BowlConfig:
    xyz_base_m: Vec3
    yaw_deg: float


@dataclass(frozen=True)
class FixedWorkspaceLayout:
    frame: str
    robot_world: Pose2D
    table_surface_z_world_m: float
    cube: CubeConfig
    bowl: BowlConfig
    source: str  # file path this was loaded from, or "<legacy-default>"


@dataclass(frozen=True)
class ResolvedObjectPose:
    """A fully-resolved object pose, ready to write onto a USD prim, plus
    enough provenance to print a `[LAYOUT]` startup log line."""

    xyz_world_m: Vec3
    yaw_world_deg: float
    xyz_base_m: Vec3 | None  # None when a --*_pos CLI override bypassed the base-frame transform
    position_source: str  # "cli_override" | "layout"
    yaw_source: str  # "cli_override" | "layout"


# ---------------------------------------------------------------------------
# Validation helpers -- every one raises LayoutError with a field-specific
# message; nothing here silently substitutes a default for a bad value.
# ---------------------------------------------------------------------------


def _require_field(d: dict, key: str, field_name: str):
    if key not in d:
        raise LayoutError(f"Missing required field: {field_name}")
    return d[key]


def _require_object(value, field_name: str) -> dict:
    if not isinstance(value, dict):
        raise LayoutError(f"{field_name} must be a JSON object, got: {type(value).__name__}")
    return value


def _is_finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _require_vec3(value, field_name: str) -> Vec3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise LayoutError(f"{field_name} must be an array of exactly 3 numbers, got: {value!r}")
    if not all(_is_finite_number(v) for v in value):
        raise LayoutError(f"{field_name} must contain 3 finite numbers, got: {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


def _require_finite_scalar(value, field_name: str) -> float:
    if not _is_finite_number(value):
        raise LayoutError(f"{field_name} must be a finite number, got: {value!r}")
    return float(value)


def _require_positive_scalar(value, field_name: str) -> float:
    v = _require_finite_scalar(value, field_name)
    if v <= 0.0:
        raise LayoutError(f"{field_name} must be > 0, got: {v}")
    return v


def validate_yaw_deg(value, field_name: str = "yaw_deg") -> float:
    """Shared validator for both layout-JSON yaw fields and CLI --*_yaw_deg
    overrides (argparse's type=float happily accepts 'inf'/'nan', which this
    rejects with a clear message instead of propagating a non-finite yaw)."""
    return _require_finite_scalar(value, field_name)


def parse_layout_dict(data, source: str) -> FixedWorkspaceLayout:
    if not isinstance(data, dict):
        raise LayoutError(f"Layout root must be a JSON object, got: {type(data).__name__}")

    version = _require_field(data, "version", "version")
    if version != SUPPORTED_VERSION:
        raise LayoutError(f"Unsupported layout version: {version!r} (this build supports version {SUPPORTED_VERSION})")

    frame = _require_field(data, "frame", "frame")
    if frame != SUPPORTED_FRAME:
        raise LayoutError(f"Unsupported frame: {frame!r} (this build only supports frame {SUPPORTED_FRAME!r})")

    robot_world = _require_object(_require_field(data, "robot_world", "robot_world"), "robot_world")
    robot_xyz = _require_vec3(_require_field(robot_world, "xyz_m", "robot_world.xyz_m"), "robot_world.xyz_m")
    robot_yaw = validate_yaw_deg(robot_world.get("yaw_deg", 0.0), "robot_world.yaw_deg")

    table = _require_object(_require_field(data, "table", "table"), "table")
    table_z = _require_finite_scalar(
        _require_field(table, "surface_z_world_m", "table.surface_z_world_m"), "table.surface_z_world_m"
    )

    cube = _require_object(_require_field(data, "cube", "cube"), "cube")
    cube_xyz = _require_vec3(_require_field(cube, "xyz_base_m", "cube.xyz_base_m"), "cube.xyz_base_m")
    cube_yaw = validate_yaw_deg(cube.get("yaw_deg", 0.0), "cube.yaw_deg")
    cube_size = _require_vec3(_require_field(cube, "size_m", "cube.size_m"), "cube.size_m")
    for axis_name, axis_value in zip("xyz", cube_size):
        if axis_value <= 0.0:
            raise LayoutError(f"cube.size_m.{axis_name} must be > 0, got: {axis_value}")
    cube_mass = _require_positive_scalar(_require_field(cube, "mass_kg", "cube.mass_kg"), "cube.mass_kg")
    cube_rest_on_table = bool(cube.get("rest_on_table", False))
    if cube_rest_on_table and cube_xyz[2] != 0.0:
        raise LayoutError(
            "cube.rest_on_table=true together with a nonzero cube.xyz_base_m[2] is ambiguous "
            f"(got z={cube_xyz[2]}) -- set cube.xyz_base_m[2] to 0.0 when rest_on_table is enabled; "
            "Z is then derived from table.surface_z_world_m + cube.size_m[2]/2 instead."
        )

    bowl = _require_object(_require_field(data, "bowl", "bowl"), "bowl")
    bowl_xyz = _require_vec3(_require_field(bowl, "xyz_base_m", "bowl.xyz_base_m"), "bowl.xyz_base_m")
    bowl_yaw = validate_yaw_deg(bowl.get("yaw_deg", 0.0), "bowl.yaw_deg")

    return FixedWorkspaceLayout(
        frame=frame,
        robot_world=Pose2D(xyz_m=robot_xyz, yaw_deg=robot_yaw),
        table_surface_z_world_m=table_z,
        cube=CubeConfig(
            xyz_base_m=cube_xyz, yaw_deg=cube_yaw, size_m=cube_size, mass_kg=cube_mass, rest_on_table=cube_rest_on_table
        ),
        bowl=BowlConfig(xyz_base_m=bowl_xyz, yaw_deg=bowl_yaw),
        source=source,
    )


def load_layout(path: str) -> FixedWorkspaceLayout:
    """Load + validate a layout JSON file. Raises LayoutError (never returns
    a partially-valid layout) so callers can fail fast before Kit boots."""
    if not os.path.isfile(path):
        raise LayoutError(f"Layout file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        raw = f.read()
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise LayoutError(f"Layout file is not valid JSON: {path}: {exc}") from exc
    return parse_layout_dict(data, source=path)


# ---------------------------------------------------------------------------
# Frame math: T_world_obj = T_world_base @ T_base_obj, planar (yaw + XYZ).
# ---------------------------------------------------------------------------


def rotate_xy_deg(xy: tuple[float, float], yaw_deg: float) -> tuple[float, float]:
    """Rotate a 2D (x, y) point about the origin by yaw_deg, right-handed
    about +Z (CCW looking down -Z from above)."""
    x, y = xy
    theta = math.radians(yaw_deg)
    c, s = math.cos(theta), math.sin(theta)
    return (x * c - y * s, x * s + y * c)


def base_to_world_xyz(xyz_base_m: Vec3, robot_world: Pose2D) -> Vec3:
    """T_world_base @ xyz_base_m -- rotate the base-relative offset by the
    base's own yaw, then translate by the base's world position."""
    rx, ry = rotate_xy_deg((xyz_base_m[0], xyz_base_m[1]), robot_world.yaw_deg)
    rz = xyz_base_m[2]
    return (rx + robot_world.xyz_m[0], ry + robot_world.xyz_m[1], rz + robot_world.xyz_m[2])


def yaw_deg_to_quat_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    """Pure rotation about +Z as a (w, x, y, z) unit quaternion (roll/pitch
    zero) -- matches isaaclab.utils.math's wxyz convention, see
    utils/geometry.py's module docstring."""
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def cube_world_pose(layout: FixedWorkspaceLayout) -> tuple[Vec3, float]:
    """Returns (xyz_world_m, yaw_world_deg) for the cube, honoring
    cube.rest_on_table (which overrides only the Z component)."""
    xyz_world = base_to_world_xyz(layout.cube.xyz_base_m, layout.robot_world)
    if layout.cube.rest_on_table:
        xyz_world = (xyz_world[0], xyz_world[1], layout.table_surface_z_world_m + layout.cube.size_m[2] / 2.0)
    yaw_world = layout.robot_world.yaw_deg + layout.cube.yaw_deg
    return xyz_world, yaw_world


def bowl_world_pose(layout: FixedWorkspaceLayout) -> tuple[Vec3, float]:
    """Returns (xyz_world_m, yaw_world_deg) for the bowl."""
    xyz_world = base_to_world_xyz(layout.bowl.xyz_base_m, layout.robot_world)
    yaw_world = layout.robot_world.yaw_deg + layout.bowl.yaw_deg
    return xyz_world, yaw_world


def cube_table_contact_gap_m(layout: FixedWorkspaceLayout, cube_world_z: float) -> float:
    """Signed gap (meters) between the cube's *resolved* world Z and where
    its bottom face would sit exactly on the configured table surface --
    0 means flush contact, positive means floating above the table.
    A consistency check only (see leader_arm_teleop_raw_isaacsim.py's
    startup warning) -- never used to silently rewrite a configured pose."""
    expected_center_z = layout.table_surface_z_world_m + layout.cube.size_m[2] / 2.0
    return cube_world_z - expected_center_z


# ---------------------------------------------------------------------------
# CLI override resolution (pure -- Gf/pxr conversion happens in the runtime
# script). Precedence, most to least specific: CLI override > layout JSON >
# legacy constants/defaults (the legacy tier is baked into default_layout()
# below, so by the time a FixedWorkspaceLayout exists, only two tiers --
# "layout" and "cli_override" -- remain to resolve here).
# ---------------------------------------------------------------------------


def resolve_cube_pose(
    layout: FixedWorkspaceLayout,
    cli_pos_xyz_m: Vec3 | None = None,
    cli_yaw_deg: float | None = None,
) -> ResolvedObjectPose:
    layout_xyz, layout_yaw = cube_world_pose(layout)
    if cli_pos_xyz_m is not None:
        xyz_world = cli_pos_xyz_m
        base_xyz = None  # world position no longer expressed relative to the base frame
        position_source = "cli_override"
    else:
        xyz_world = layout_xyz
        base_xyz = layout.cube.xyz_base_m
        position_source = "layout"

    if cli_yaw_deg is not None:
        yaw_world = layout.robot_world.yaw_deg + cli_yaw_deg
        yaw_source = "cli_override"
    else:
        yaw_world = layout_yaw
        yaw_source = "layout"

    return ResolvedObjectPose(xyz_world, yaw_world, base_xyz, position_source, yaw_source)


def resolve_bowl_pose(
    layout: FixedWorkspaceLayout,
    cli_pos_xyz_m: Vec3 | None = None,
    cli_yaw_deg: float | None = None,
) -> ResolvedObjectPose:
    layout_xyz, layout_yaw = bowl_world_pose(layout)
    if cli_pos_xyz_m is not None:
        xyz_world = cli_pos_xyz_m
        base_xyz = None
        position_source = "cli_override"
    else:
        xyz_world = layout_xyz
        base_xyz = layout.bowl.xyz_base_m
        position_source = "layout"

    if cli_yaw_deg is not None:
        yaw_world = layout.robot_world.yaw_deg + cli_yaw_deg
        yaw_source = "cli_override"
    else:
        yaw_world = layout_yaw
        yaw_source = "layout"

    return ResolvedObjectPose(xyz_world, yaw_world, base_xyz, position_source, yaw_source)


# ---------------------------------------------------------------------------
# Legacy-compatible default: reproduces leader_arm_teleop_raw_isaacsim.py's
# pre-layout REAL_CUBE_POS/REAL_BOWL_POS/ROBOT_POS world constants exactly
# when no --layout/--cube_pos/--bowl_pos is given. cube/bowl xyz_base_m below
# are computed by inverse-transforming those already-measured world
# constants through LEGACY_ROBOT_WORLD_POS at yaw=0 (pure coordinate algebra
# on an existing measurement, not a new/fabricated one) -- see
# docs/hardcoded-real2sim-implementation-receipt.md for the derivation.
# ---------------------------------------------------------------------------

# Measured from the real workspace on 2026-08-19 (tape measure from the
# robot base's two feet). Re-measure and update if the table/robot/objects
# move -- these were a one-time snapshot, not tracked. Bowl X sign was a
# provisional guess pending a live-viewport check when originally measured;
# carried forward unchanged -- confirm it still matches before trusting it
# further (see docs/hardcoded-real2sim-implementation-receipt.md).
LEGACY_ROBOT_WORLD_POS: Vec3 = (0.0, 0.3, 0.72)
LEGACY_CUBE_WORLD_POS: Vec3 = (0.0, 0.047, 0.7754)
LEGACY_BOWL_WORLD_POS: Vec3 = (0.153, 0.047, 0.7500)
LEGACY_TABLE_SURFACE_Z_WORLD: float = 0.7504
LEGACY_CUBE_MASS_KG: float = 0.05
# Real cube side length 0.053m (5.3cm). real-to-sim.usd's AWSBuilderCube_Geo
# mesh was authored at 0.05m (confirmed by direct inspection: 8 points at
# exactly +/-0.025 on every axis) -- corrected here, applied at runtime as a
# uniform scale on that mesh's existing xformOp:scale (see
# AUTHORED_CUBE_GEO_SIZE_M below), not by hand-editing the checked-in USD asset.
LEGACY_CUBE_SIZE_M: Vec3 = (0.053, 0.053, 0.053)

# The cube geometry's authored (pre-correction) side length in
# real-to-sim.usd, used only to compute the runtime scale factor
# size_m[i] / AUTHORED_CUBE_GEO_SIZE_M for AWSBuilderCube_Geo's
# xformOp:scale. Not a layout-configurable value -- it's a fact about the
# shipped mesh, re-verify by direct USD inspection if that mesh is ever
# replaced.
AUTHORED_CUBE_GEO_SIZE_M: float = 0.05


def default_layout() -> FixedWorkspaceLayout:
    cube_base = tuple(w - r for w, r in zip(LEGACY_CUBE_WORLD_POS, LEGACY_ROBOT_WORLD_POS))
    bowl_base = tuple(w - r for w, r in zip(LEGACY_BOWL_WORLD_POS, LEGACY_ROBOT_WORLD_POS))
    return FixedWorkspaceLayout(
        frame=SUPPORTED_FRAME,
        robot_world=Pose2D(xyz_m=LEGACY_ROBOT_WORLD_POS, yaw_deg=0.0),
        table_surface_z_world_m=LEGACY_TABLE_SURFACE_Z_WORLD,
        cube=CubeConfig(
            xyz_base_m=cube_base,
            yaw_deg=0.0,
            size_m=LEGACY_CUBE_SIZE_M,
            mass_kg=LEGACY_CUBE_MASS_KG,
            rest_on_table=False,
        ),
        bowl=BowlConfig(xyz_base_m=bowl_base, yaw_deg=0.0),
        source="<legacy-default>",
    )
