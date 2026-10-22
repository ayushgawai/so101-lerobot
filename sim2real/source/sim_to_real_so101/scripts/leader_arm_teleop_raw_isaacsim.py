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
"""Real leader-arm teleop against raw Isaac Sim 6.0.1 -- NO Isaac Lab.

Same physics setup and platform constraint as keyboard_agent_raw_isaacsim.py
(see that file's module docstring and docs/aws-cube-to-bowl-teleop-plan.md
Sec.5) -- this is that script's keyboard jog replaced with a physical SO-101
leader arm read over serial, per the plan's Sec.5 "not yet done" list:

1. Reads `robot.get_action()` from a LeRobotSO101Interface(kind="leader")
   each tick instead of keyboard deltas.
2. The minimal teleop adapter explicitly selects normalized LeRobot units
   for both arms, then maps the accepted command to USD joint-drive degrees.
3. Requires `lerobot` (this repo's root package, `.[so101]` extra)
   installed into whatever Python runs this script -- NOT present in a
   stock Isaac Sim python by default.
4. HF_LEROBOT_CALIBRATION is pointed at this repo's calibration/ folder
   (not the ~/.cache default) so `id="my_so_arm"` resolves to the existing
   calibration/teleoperators/so_leader/my_so_arm.json.

Run with a plain Isaac Sim Python that has lerobot installed (see repo docs
for the install command), by direct file path -- NOT `-m` (same reasoning
as keyboard_agent_raw_isaacsim.py):
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\leader_arm_teleop_raw_isaacsim.py

Scope (deliberately minimal, matching the keyboard version): direct
leader-arm-to-joint-target passthrough, plus a keyboard 'R' to reset the
cube/bowl to their original positions (joint targets aren't reset-able here
the way the keyboard script does it -- they're driven live by the leader arm
every tick, not held state). No recording, no grasp/placement detection.

Optional real-follower mirror (Phase 1+2 of
docs/dual-teleop-sim-and-follower-plan.md): pass --follower_port (e.g. COM3)
to also mirror the same leader reading, same tick, to a real SO-101 follower
via LeRobotSO101Interface(kind="follower").send_action(). Unset (default):
behavior is unchanged from the sim-only script described above.

Real cube/bowl positions (see docs/object-pose-mirroring-plan.md for the
camera-tracking approach this replaces -- dropped in favor of a one-time
manual measurement, since the objects don't move between episodes and a
tracker's calibration cost/OOD risk isn't worth it for this purpose):
a fixed-layout config (utils/fixed_workspace.py) expresses the cube/bowl
poses relative to the SO-101 base mounting point, converted to Isaac world
poses at startup -- pass --layout <path> to point at a measured layout JSON
(see calibration/workspaces/aws_cube_bowl_fixed.json and
docs/aws-cube-to-bowl-run-guide.md), or omit it to fall back to an in-code
legacy layout equivalent to this file's old hardcoded world-frame constants.
--cube_pos / --bowl_pos (world-frame x,y,z) and --cube_yaw_deg /
--bowl_yaw_deg still work exactly as before, as CLI overrides on top of
whichever layout is in effect. The cube stays fully dynamic (grippable,
physics-driven) throughout -- only its *starting* pose is overridden, not
puppeteered every tick.
"""
import argparse
import json
import math
import os
import sys
import time

# Make sim_to_real_so101 importable without requiring an editable pip install
# in whatever Python this script is run with (matches
# keyboard_agent_raw_isaacsim.py -- this repo's package is only
# pip-installed into the Isaac-Lab venvs, not into a plain Isaac Sim python).
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

# Pure config/math (json/math/dataclasses only) -- deliberately importable
# before Kit boots, unlike sim_to_real_so101.utils.keyboard/etc below, which
# transitively need omni/pxr and are imported only after SimulationApp().
from sim_to_real_so101.utils import fixed_workspace  # noqa: E402
from sim_to_real_so101.utils.wrist_roll_alignment import WristRollAlignment  # noqa: E402
from sim_to_real_so101.utils.dual_teleop_control import DualActionGate  # noqa: E402
from sim_to_real_so101.utils.teleop_hardware import (  # noqa: E402
    TeleopSO101Interface as LeRobotSO101Interface, connect_calibrated, validate_calibration,
)

parser = argparse.ArgumentParser(description="Raw Isaac Sim SO-101 leader-arm teleop agent (no Isaac Lab).")
parser.add_argument("--scene_profile", choices=("legacy", "codex"), default="legacy")
parser.add_argument("--usd_path", default=None, help="Override the profile's USD stage.")
parser.add_argument("--start_paused", action="store_true", help="Hold initial follower pose until F8; F9 pauses both outputs.")
parser.add_argument("--startup_mode", choices=("match", "ramp"), default="match",
                    help="match requires aligned arm poses; ramp lets F8 converge from the measured follower pose at 10 normalized units/s.")
parser.add_argument("--require_camera_alignment", action="store_true", help="Require a successful measured dual-camera layout before connecting motors.")
parser.add_argument("--show_cameras", action="store_true", help="Show fixed and wrist camera views in Kit.")
parser.add_argument("--camera_indices", type=int, nargs=2, default=(1, 0), metavar=("FIXED", "WRIST"))
parser.add_argument("--wrist_roll_offset_deg", type=float, default=-90.0, help="Existing wrist correction; verify against the real arm before grasping.")
parser.add_argument("--control_hz", type=float, default=30.0)
parser.add_argument("--control_timeout_s", type=float, default=2.0,
                    help="Pause and require F8 after this loop gap in seconds. Shorter delays continue with a capped command step (default: 2).")
parser.add_argument("--preflight_only", action="store_true", help="Validate files/configuration without starting Kit or opening hardware.")
parser.add_argument("--sim_test_seconds", type=float, default=0., help="Finite physics smoke test; never opens serial ports or cameras.")
parser.add_argument("--headless", action="store_true", default=False, help="Run without a viewport window.")
parser.add_argument("--port", type=str, default=os.getenv("TELEOP_PORT", "COM4"), help="Leader arm serial port.")
parser.add_argument(
    "--robot_id",
    type=str,
    default=os.getenv("TELEOP_ID", "my_so_arm"),
    help="Leader arm calibration id (must match a file under calibration/teleoperators/so_leader/).",
)
parser.add_argument(
    "--calibration_dir",
    type=str,
    default=os.path.join(os.path.dirname(_REPO_ROOT_DIR), "calibration"),
    help="Root calibration directory (expects <this>/teleoperators/so_leader/<robot_id>.json). "
    "Sets HF_LEROBOT_CALIBRATION -- must be set before `lerobot` is imported.",
)
parser.add_argument(
    "--follower_port",
    type=str,
    default=None,
    help="Real follower arm serial port (e.g. COM3). Unset (default) = today's sim-only behavior, "
    "nothing else changed. When given, the same leader reading driving the sim is also mirrored "
    "to a real follower on this port each tick -- see docs/dual-teleop-sim-and-follower-plan.md.",
)
parser.add_argument(
    "--follower_robot_id",
    type=str,
    default="my_so_arm",
    help="Follower arm calibration id (must match a file under calibration/robots/so_follower/). "
    "Only used when --follower_port is given.",
)
parser.add_argument(
    "--cube_pos",
    type=str,
    default=None,
    help="Override the cube's resolved position as 'x,y,z' in meters (world frame, absolute -- "
    "bypasses the --layout base-frame math entirely). Unset (default) = use --layout (or the "
    "legacy default if --layout is unset).",
)
parser.add_argument(
    "--bowl_pos",
    type=str,
    default=None,
    help="Override the bowl's resolved position as 'x,y,z' in meters (world frame, absolute -- "
    "bypasses the --layout base-frame math entirely). Unset (default) = use --layout (or the "
    "legacy default if --layout is unset).",
)
parser.add_argument(
    "--layout",
    type=str,
    default=None,
    help="Path to a fixed-workspace layout JSON (e.g. calibration/workspaces/aws_cube_bowl_fixed.json) "
    "defining the cube/bowl poses relative to the SO-101 base frame. Unset (default) = an in-code "
    "legacy layout that reproduces this file's old hardcoded REAL_CUBE_POS/REAL_BOWL_POS world "
    "constants exactly. See docs/aws-cube-to-bowl-run-guide.md for the schema and measurement "
    "procedure. Precedence: --cube_pos/--bowl_pos/--cube_yaw_deg/--bowl_yaw_deg (if given) > this "
    "layout > the legacy default.",
)
parser.add_argument(
    "--cube_yaw_deg",
    type=float,
    default=None,
    help="Override the cube's yaw (degrees, about +Z, base-frame convention) from the layout. "
    "Unset (default) = use the layout's cube.yaw_deg (0.0 if no --layout given).",
)
parser.add_argument(
    "--bowl_yaw_deg",
    type=float,
    default=None,
    help="Override the bowl's yaw (degrees, about +Z, base-frame convention) from the layout. "
    "Unset (default) = use the layout's bowl.yaw_deg (0.0 if no --layout given).",
)
args_cli = parser.parse_args()
if not math.isfinite(args_cli.control_hz) or args_cli.control_hz <= 0:
    parser.error("--control_hz must be positive and finite")
if not math.isfinite(args_cli.control_timeout_s) or args_cli.control_timeout_s <= 0:
    parser.error("--control_timeout_s must be positive and finite")
if args_cli.control_timeout_s <= 1/args_cli.control_hz:
    parser.error("--control_timeout_s must exceed the nominal control period (1 / --control_hz)")
if not math.isfinite(args_cli.sim_test_seconds) or args_cli.sim_test_seconds < 0:
    parser.error("--sim_test_seconds must be finite and nonnegative")
if not math.isfinite(args_cli.wrist_roll_offset_deg):
    parser.error("--wrist_roll_offset_deg must be finite")
SIM_TEST = args_cli.sim_test_seconds > 0
if args_cli.startup_mode == "ramp" and not SIM_TEST:
    args_cli.start_paused = True
if args_cli.scene_profile == "codex" and args_cli.follower_port and not SIM_TEST:
    args_cli.start_paused = True
if args_cli.follower_port and args_cli.follower_port.upper() == args_cli.port.upper():
    parser.error("Leader and follower ports must differ")
if args_cli.headless and args_cli.start_paused and not SIM_TEST:
    parser.error("--start_paused requires a visible Kit window for F8/F9 controls")
if args_cli.scene_profile == "codex" and args_cli.layout is None:
    args_cli.layout = os.path.join(_REPO_ROOT_DIR, "calibration", "workspaces", "codex_pick_place_photo.json")
if args_cli.require_camera_alignment:
    if any(getattr(args_cli, name) is not None for name in ("cube_pos", "bowl_pos", "cube_yaw_deg", "bowl_yaw_deg")):
        parser.error("Position overrides invalidate camera registration; update the measured layout instead")
    if args_cli.layout is None or not os.path.isfile(args_cli.layout):
        parser.error("Run align_dual_camera_workspace.py and pass its measured --layout first")
    with open(args_cli.layout, encoding="utf-8") as stream:
        alignment = json.load(stream).get("alignment", {})
    if alignment.get("status") != "passed":
        parser.error("The layout is nominal/unmeasured; a passed two-camera registration is required")
    if args_cli.follower_port and alignment.get("source_mode") != "live":
        parser.error("Real follower operation requires live registration, not offline test frames")

# Fail fast on a bad --robot_id/--calibration_dir (e.g. a typo) before paying
# for Kit's ~30s+ boot below -- lerobot's own error for a missing calibration
# file only surfaces later, inside connect(), with a much less obvious
# traceback originating deep in its serial handshake code.
_CALIBRATION_FILE = os.path.join(
    args_cli.calibration_dir, "teleoperators", "so_leader", f"{args_cli.robot_id}.json"
)
if not SIM_TEST and not os.path.isfile(_CALIBRATION_FILE):
    print(f"[ERROR]: Calibration file not found: {_CALIBRATION_FILE}", file=sys.stderr)
    print(
        "[ERROR]: Check --robot_id / --calibration_dir (or TELEOP_ID / TELEOP_PORT env vars).",
        file=sys.stderr,
    )
    sys.exit(1)

if args_cli.follower_port is not None and not SIM_TEST:
    _FOLLOWER_CALIBRATION_FILE = os.path.join(
        args_cli.calibration_dir, "robots", "so_follower", f"{args_cli.follower_robot_id}.json"
    )
    if not os.path.isfile(_FOLLOWER_CALIBRATION_FILE):
        print(f"[ERROR]: Follower calibration file not found: {_FOLLOWER_CALIBRATION_FILE}", file=sys.stderr)
        print("[ERROR]: Check --follower_robot_id / --calibration_dir.", file=sys.stderr)
        sys.exit(1)

args_cli.calibration_dir = os.path.abspath(args_cli.calibration_dir)
try:
    if not SIM_TEST:
        validate_calibration(_CALIBRATION_FILE)
        if args_cli.follower_port:
            validate_calibration(_FOLLOWER_CALIBRATION_FILE)
except ValueError as exc:
    parser.error(str(exc))

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.abspath(args_cli.usd_path or os.path.join(
    _DEMO_DIR, "codex-pick-place-table.usd" if args_cli.scene_profile == "codex" else "real-to-sim.usd"
))
if not os.path.isfile(REAL_TO_SIM_USD):
    parser.error(f"USD scene does not exist: {REAL_TO_SIM_USD}")

def _parse_vec3_str(arg_name: str, value: str | None) -> tuple[float, float, float] | None:
    """Fail fast on a malformed --cube_pos/--bowl_pos before paying for Kit's
    ~30s+ boot below, same reasoning as the calibration-file checks above.
    Returns the parsed (x, y, z) tuple, or None if value is None."""
    if value is None:
        return None
    parts = value.split(",")
    if len(parts) != 3:
        print(
            f"[ERROR]: --{arg_name} must be 'x,y,z' (three comma-separated numbers), got: {value!r}",
            file=sys.stderr,
        )
        sys.exit(1)
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        print(f"[ERROR]: --{arg_name} must be numeric 'x,y,z', got: {value!r}", file=sys.stderr)
        sys.exit(1)


_CLI_CUBE_POS = _parse_vec3_str("cube_pos", args_cli.cube_pos)
_CLI_BOWL_POS = _parse_vec3_str("bowl_pos", args_cli.bowl_pos)

# Load + validate the fixed-workspace layout (pure config/math, no
# isaacsim/omni import -- see utils/fixed_workspace.py) and resolve the
# final cube/bowl poses now, before Kit boots, so a bad --layout or
# --cube_yaw_deg/--bowl_yaw_deg fails fast like the checks above instead of
# surfacing only after the ~30s+ Kit boot. Precedence: CLI override (--
# cube_pos/--bowl_pos/--cube_yaw_deg/--bowl_yaw_deg) > --layout JSON >
# in-code legacy default (fixed_workspace.default_layout(), which
# reproduces this file's old hardcoded REAL_CUBE_POS/REAL_BOWL_POS/
# ROBOT_POS constants exactly).
try:
    if args_cli.layout is not None:
        WORKSPACE_LAYOUT = fixed_workspace.load_layout(args_cli.layout)
    else:
        WORKSPACE_LAYOUT = fixed_workspace.default_layout()
    if args_cli.cube_yaw_deg is not None:
        fixed_workspace.validate_yaw_deg(args_cli.cube_yaw_deg, "--cube_yaw_deg")
    if args_cli.bowl_yaw_deg is not None:
        fixed_workspace.validate_yaw_deg(args_cli.bowl_yaw_deg, "--bowl_yaw_deg")
except fixed_workspace.LayoutError as exc:
    print(f"[ERROR]: {exc}", file=sys.stderr)
    sys.exit(1)

RESOLVED_CUBE_POSE = fixed_workspace.resolve_cube_pose(
    WORKSPACE_LAYOUT, cli_pos_xyz_m=_CLI_CUBE_POS, cli_yaw_deg=args_cli.cube_yaw_deg
)
RESOLVED_BOWL_POSE = fixed_workspace.resolve_bowl_pose(
    WORKSPACE_LAYOUT, cli_pos_xyz_m=_CLI_BOWL_POS, cli_yaw_deg=args_cli.bowl_yaw_deg
)

# Must happen before any `lerobot` import (including transitively, via
# sim_to_real_so101.utils.lerobot_interface below) -- lerobot.utils.constants
# reads this env var at import time to compute HF_LEROBOT_CALIBRATION, and
# that constant is what resolves calibration/teleoperators/so_leader/<id>.json.
os.environ["HF_LEROBOT_CALIBRATION"] = args_cli.calibration_dir

if args_cli.preflight_only:
    print(f"[PASS] Scene file: {REAL_TO_SIM_USD}")
    print(f"[PASS] Calibration root: {args_cli.calibration_dir}")
    print(f"[PASS] Leader: {args_cli.port}, id={args_cli.robot_id}; follower: {args_cli.follower_port}, id={args_cli.follower_robot_id}")
    print(f"[PASS] Layout: {WORKSPACE_LAYOUT.source}; cube={RESOLVED_CUBE_POSE.xyz_world_m}; bowl={RESOLVED_BOWL_POSE.xyz_world_m}")
    print(f"[PASS] Control: {args_cli.control_hz:g} Hz; hard gap limit={args_cli.control_timeout_s:g}s; maximum command time step={min(1/args_cli.control_hz, 1/30):.4f}s")
    print("[PASS] File/config preflight only; hardware, EEPROM and Isaac runtime have not been checked.")
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args_cli.headless})

"""Rest everything follows -- Kit is booted, pxr/omni/carb are importable now."""

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import carb  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from sim_to_real_so101.utils.keyboard import KeyboardControl  # noqa: E402
from sim_to_real_so101.utils.physics_material import apply_friction_material  # noqa: E402
from sim_to_real_so101.utils.paper_cube import is_rubber_cube, is_soft_cube, soft_cube_world_bounds, restore_soft_cube_points  # noqa: E402
from sim_to_real_so101.utils.scene_reset import (  # noqa: E402
    ensure_translate_orient_ops,
    restore_prim_pose,
    snapshot_xform_ops,
)
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = args_cli.usd_path or os.path.join(
    _DEMO_DIR, "codex-pick-place-table.usd" if args_cli.scene_profile == "codex" else "real-to-sim.usd"
)

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
AWS_CUBE_PRIM_PATH = "/World/AWSBuilderCube"
# See keyboard_agent_raw_isaacsim.py's identical constant -- friction is a
# collision-shape property (PhysicsMaterialAPI), not a rigid-body property.
AWS_CUBE_COLLISION_MESH_PATH = "/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo"
PAPER_BOWL_PRIM_PATH = "/World/PaperBowl"
if args_cli.scene_profile == "codex":
    AWS_CUBE_PRIM_PATH = "/World/Cube"
    AWS_CUBE_COLLISION_MESH_PATH = "/World/Cube/Collision"
    PAPER_BOWL_PRIM_PATH = "/World/Container"

# Cube/bowl pose, size, and mass now come from WORKSPACE_LAYOUT/
# RESOLVED_CUBE_POSE/RESOLVED_BOWL_POSE (resolved above, before Kit boot --
# see utils/fixed_workspace.py) instead of the hardcoded REAL_CUBE_POS/
# REAL_BOWL_POS/AWS_CUBE_MASS_KG world-frame constants this file used to
# carry directly.
TABLE_CONTACT_WARN_TOLERANCE_M = 0.005

# See keyboard_agent_raw_isaacsim.py -- gripper pad collision shapes.
GRIPPER_COLLISION_PATH = "/World/SO_ARM101_USD/gripper/collisions"
JAW_COLLISION_PATH = "/World/SO_ARM101_USD/jaw/collisions"

# See keyboard_agent_raw_isaacsim.py -- rigid PVC-like friction for the cube
# (matches PhysicsMaterialAPI on real-to-sim.usd's AWSBuilderCube_Geo).
AWS_CUBE_STATIC_FRICTION = 0.45
AWS_CUBE_DYNAMIC_FRICTION = 0.40
AWS_CUBE_RESTITUTION = 0.0

# See keyboard_agent_raw_isaacsim.py -- FDM 3D-printed PLA/PETG friction for
# the jaw/gripper pads (matches PhysicsMaterialAPI on real-to-sim.usd).
GRIPPER_STATIC_FRICTION = 0.40
GRIPPER_DYNAMIC_FRICTION = 0.35
GRIPPER_RESTITUTION = 0.0

# Isaac-Lab-tuned actuator gains, used AS-IS unconverted on the raw
# PhysicsDriveAPI attributes -- see keyboard_agent_raw_isaacsim.py's
# JOINT_GAINS for the full empirical justification (unconverted values
# produce correct grasp torque; the naive pi/180-scaled version doesn't).
# Dict order here also doubles as the action-tensor order expected from
# LeRobotSO101Interface.get_mapped_actions_vectorized() -- both this and
# the Isaac-Lab task's ActionsCfg.joint_positions.joint_names use the same
# positional correspondence to SO101_JOINT_ORDER (shoulder_pan, shoulder_lift,
# elbow_flex, wrist_flex, wrist_roll, gripper), not a name-based mapping.
JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    # effort_limit=3 (not 30): commanding full close against AWSBuilderCube is
    # an unreachable target (pads stop at cube width), so the PD drive saturates
    # at maxForce every tick. 30 N*m ejects the cube; 3 N*m still grips (~250x
    # the holding torque a 50g cube needs) without launching it. Same value as
    # keyboard_agent_raw_isaacsim.py / replay_act_dataset_to_sim.py /
    # test_grasp_dynamics.py.
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=3),
}
JOINT_ORDER = list(JOINT_GAINS.keys())

# Wrist-roll leader-to-sim zero-pose alignment (2026-08-24 fix) -- see
# utils/wrist_roll_alignment.py's module docstring for the full root-cause
# writeup. Short version: get_mapped_actions_vectorized()'s per-joint scale
# passes exactly through raw=0 -> 0 deg for every joint, which silently
# assumes the leader's calibration zero coincides with the USD joint's own
# authored 0 deg pose. That holds for the other five joints but not for
# Wrist_Roll (confirmed by direct observation: physical wrist upright vs.
# simulated wrist rotated ~90 deg at near-zero leader/sim readings), so this
# applies an explicit, configurable correction to Wrist_Roll ONLY, after the
# scale and before joint-limit clamping -- every other joint's mapping is
# unchanged.
#
# zero_offset_deg=-90.0 set 2026-08-24 per explicit user request (+90.0 was
# tried first and changed to -90.0), on top of direction_sign=+1.0 (real and
# sim confirmed rotating the same side live on hardware -- see the
# neutral-pose and horizontal-quarter-turn checks in the session that
# produced this value). NOTE: those same live checks measured offset=0.0 as
# the matching value at that moment (recalibrated arm); this -90.0 was
# requested afterward without a re-verified live mismatch behind it. If the
# simulated wrist looks off by ~90 deg at neutral after this change, that
# confirms this value was wrong for the current calibration -- revert to
# 0.0, or re-derive per utils/wrist_roll_alignment.py's docstring.
WRIST_ROLL_ALIGNMENT = WristRollAlignment(direction_sign=1.0, zero_offset_deg=args_cli.wrist_roll_offset_deg)

# Phase 2 timing measurement -- how often to flush the tick/send-time summary
# (see the main loop). Deliberately periodic, not per-tick, per the plan's
# "no per-tick spam" requirement.
STATS_PRINT_INTERVAL_S = 5.0

# Follower startup sync (dual-teleop-sim-and-follower-plan.md's "Follower
# robot safety" follow-up): without this, connecting a real follower snaps
# it straight to the leader's current position with no ramp the instant
# send_action() is first called -- SO101FollowerConfig.max_relative_target
# defaults to None (confirmed: LeRobotSO101Interface.make_cfg() never sets
# it), so nothing in lerobot itself limits that first jump. Ramping over a
# short, fixed duration instead is a conservative mitigation, not a
# guarantee -- it does not replace positioning the physical leader near the
# follower's actual pose before connecting.
FOLLOWER_STARTUP_SYNC_DURATION_S = 1.5
FOLLOWER_STARTUP_SYNC_STEPS = 45


def _fmt_xyz(xyz) -> str:
    return f"({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f})"


def _ramp_follower_to_leader_start(follower_robot, leader_action: dict) -> None:
    """Gradually move a just-connected follower from its current pose to the
    leader's current reading, instead of send_action()'s usual instant
    snap-to-target on the very first call. Read follower initial state ->
    read leader target -> interpolate -> hand off to normal per-tick
    mirroring, per the "Follower robot safety" requirement. Never raises --
    any failure here falls back to today's snap behavior rather than
    destabilizing the demo start."""
    try:
        obs = follower_robot.get_observation()
    except Exception as exc:
        print(f"[WARN]: Follower startup sync skipped (get_observation failed: {exc}) -- falling back to an instant snap.")
        return

    start_pos = {k: v for k, v in obs.items() if k.endswith(".pos")}
    target_pos = {k: v for k, v in leader_action.items() if k.endswith(".pos")}
    common_keys = sorted(k for k in target_pos if k in start_pos)
    if not common_keys:
        print("[WARN]: Follower startup sync skipped (no shared '.pos' keys between observation and action).")
        return

    max_delta = max(abs(target_pos[k] - start_pos[k]) for k in common_keys)
    print(
        f"[INFO]: Follower startup sync: max joint delta {max_delta:.1f}, ramping over "
        f"{FOLLOWER_STARTUP_SYNC_DURATION_S:.1f}s before normal mirroring begins."
    )
    step_dt = FOLLOWER_STARTUP_SYNC_DURATION_S / FOLLOWER_STARTUP_SYNC_STEPS
    for step in range(1, FOLLOWER_STARTUP_SYNC_STEPS + 1):
        alpha = step / FOLLOWER_STARTUP_SYNC_STEPS
        interpolated = {k: start_pos[k] + alpha * (target_pos[k] - start_pos[k]) for k in common_keys}
        try:
            follower_robot.send_action(interpolated)
        except Exception as exc:
            print(f"[WARN]: Follower startup sync aborted mid-ramp ({exc}) -- entering normal mirroring now.")
            return
        time.sleep(step_dt)
    print("[INFO]: Follower startup sync complete, entering normal mirroring.")


class DualTeleopKeys(KeyboardControl):
    def __init__(self):
        self.arm_requested = self.pause_requested = self.exit_requested = False
        super().__init__()

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "F8":
                self.arm_requested = True
                return True
            if event.input.name == "F9":
                self.pause_requested = True
                return True
            if event.input.name == "ESCAPE":
                self.exit_requested = True
                return True
        return super()._on_keyboard_event(event, *args, **kwargs)


def _connect_without_recalibration(interface, hold_current=False):
    """Check EEPROM against the supplied calibration before configuration/motion."""
    interface.init_device()
    connect_calibrated(interface.robot, hold_current=hold_current)


_CONNECTED_INTERFACES = []


def main():
    # Arm connections happen BEFORE any scene/physics/rendering setup below.
    # Root-caused via isolate_follower_connect.py / isolate_dual_arm_connect.py /
    # isolate_kit_dual_arm_connect.py (2026-08-24): the follower's connect()
    # (~42 rapid serial writes in configure()) silently killed the whole Kit
    # process with no Python traceback when it ran while the USD stage was
    # loaded and physics was playing -- reproduced identically with just Kit
    # booted (no stage) or just both arms connected (no Kit) never failed.
    # Connecting first, while Kit is idle, avoids whatever contention that
    # burst of writes hits once the render/physics/sensor pipeline is live.
    robot_iface = LeRobotSO101Interface(
        device="cpu",
        port=args_cli.port,
        id=args_cli.robot_id,
        cameras={},
        fps=30,
        kind="leader",
    )
    connected = False
    if not SIM_TEST:
        _CONNECTED_INTERFACES.append(robot_iface)
        try:
            if args_cli.start_paused:
                _connect_without_recalibration(robot_iface)
            else:
                robot_iface.init_device()
                robot_iface.connect()
            connected = True
        except Exception as exc:
            raise RuntimeError(
                f"Failed to connect to the leader arm on port={args_cli.port} "
                f"id={args_cli.robot_id}: {exc}"
            ) from exc

    # Follower mirror -- only connected when --follower_port is given (see
    # docs/dual-teleop-sim-and-follower-plan.md Phase 1). device="cuda" per
    # the plan: this machine has an NVIDIA GPU, unlike the leader interface
    # above which stays device="cpu" (unchanged, out of scope here).
    follower_iface = None
    follower_connected = False
    follower_send_failed = False
    if args_cli.follower_port is not None and not SIM_TEST:
        follower_iface = LeRobotSO101Interface(
            device="cpu",
            port=args_cli.follower_port,
            id=args_cli.follower_robot_id,
            cameras={},
            fps=30,
            kind="follower",
        )
        _CONNECTED_INTERFACES.append(follower_iface)
        try:
            if args_cli.start_paused:
                _connect_without_recalibration(follower_iface, hold_current=True)
            else:
                follower_iface.init_device()
                follower_iface.connect()
            follower_connected = True
        except Exception as exc:
            if connected:
                robot_iface.robot.disconnect()
            raise RuntimeError(
                f"Failed to connect to the follower arm on port={args_cli.follower_port} "
                f"id={args_cli.follower_robot_id}: {exc}. Check the arm is powered, plugged into the "
                "given --follower_port, and not already held open by another process."
            ) from exc
        print(f"[INFO]: Follower arm connected: port={args_cli.follower_port} id={args_cli.follower_robot_id}")

        if not args_cli.start_paused:
            _startup_leader_action = robot_iface.robot.get_action()
            _ramp_follower_to_leader_start(follower_iface.robot, _startup_leader_action)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(REAL_TO_SIM_USD)
    simulation_app.update()
    stage = usd_context.get_stage()
    if stage is None:
        raise RuntimeError(f"Could not open USD scene: {REAL_TO_SIM_USD}")
    # Runtime physics/poses live in a session layer; the authored scene stays reusable.
    stage.SetEditTarget(stage.GetSessionLayer())

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}")

    # See keyboard_agent_raw_isaacsim.py -- the raw file authors no
    # RigidBodyAPI/MassAPI on the cube.
    cube_prim = stage.GetPrimAtPath(AWS_CUBE_PRIM_PATH)
    if not cube_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_PRIM_PATH}")
    soft_cube = is_soft_cube(cube_prim)
    # Cube/gripper contact authored by the scene builder (FEM or hard rubber): keep it.
    authored_contact = soft_cube or is_rubber_cube(cube_prim)
    if not soft_cube and not cube_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    if not soft_cube and not cube_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(cube_prim)
    if not soft_cube:
        UsdPhysics.MassAPI(cube_prim).CreateMassAttr(WORKSPACE_LAYOUT.cube.mass_kg)

    cube_collision_mesh = stage.GetPrimAtPath(AWS_CUBE_COLLISION_MESH_PATH)
    if not cube_collision_mesh.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_COLLISION_MESH_PATH}")
    # A dynamic cube cannot use a triangle-mesh collider. Author the convex
    # approximation explicitly instead of relying on PhysX's error fallback.
    if not authored_contact:
        UsdPhysics.MeshCollisionAPI.Apply(cube_collision_mesh).CreateApproximationAttr("convexHull")
        apply_friction_material(
            cube_collision_mesh, AWS_CUBE_STATIC_FRICTION, AWS_CUBE_DYNAMIC_FRICTION, AWS_CUBE_RESTITUTION
        )
    else:
        kind = "Volume-deformable" if soft_cube else "Hard-rubber (rigid, compliant contact)"
        print(f"[MATERIAL]: {kind} paper/PVC cube; preserving authored friction, compliance and gripper material.", flush=True)

    # Correct the cube's geometry to the real cube's 5.3cm side length --
    # real-to-sim.usd's AWSBuilderCube_Geo mesh was authored at 5cm
    # (confirmed by direct inspection). AWSBuilderCube_Geo is a single
    # Mesh prim serving as *both* the visual and the collision shape
    # (PhysicsCollisionAPI + PhysicsMeshCollisionAPI applied directly to
    # it), so scaling its existing xformOp:scale corrects both together --
    # never just the visual mesh, and never leaves the collision shape at a
    # different size than what's rendered. Applied at runtime (like the
    # RigidBodyAPI/mass/friction patches above), not by hand-editing the
    # checked-in USD asset.
    if args_cli.scene_profile == "codex":
        cube_xform = UsdGeom.Xformable(cube_prim)
        cube_xform.AddScaleOp().Set(Gf.Vec3f(*(s/.055 for s in WORKSPACE_LAYOUT.cube.size_m)))
    else:
        cube_scale = tuple(s / fixed_workspace.AUTHORED_CUBE_GEO_SIZE_M for s in WORKSPACE_LAYOUT.cube.size_m)
        cube_collision_mesh.GetAttribute("xformOp:scale").Set(Gf.Vec3f(*cube_scale))

    for collision_path in (GRIPPER_COLLISION_PATH, JAW_COLLISION_PATH):
        collision_prim = stage.GetPrimAtPath(collision_path)
        if not collision_prim.IsValid():
            raise RuntimeError(f"Expected prim not found: {collision_path}")
        if not authored_contact:
            apply_friction_material(
                collision_prim, GRIPPER_STATIC_FRICTION, GRIPPER_DYNAMIC_FRICTION, GRIPPER_RESTITUTION
            )

    bowl_prim = stage.GetPrimAtPath(PAPER_BOWL_PRIM_PATH)
    if not bowl_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {PAPER_BOWL_PRIM_PATH}")

    # Apply the resolved cube/bowl world pose (layout JSON, base-frame
    # math, and any --cube_pos/--bowl_pos/--cube_yaw_deg/--bowl_yaw_deg CLI
    # override -- all already resolved into RESOLVED_CUBE_POSE/
    # RESOLVED_BOWL_POSE before Kit booted) before anything snapshots or
    # plays. ensure_translate_orient_ops adds an xformOp:orient if one isn't
    # already authored (AWSBuilderCube has none today) without duplicating
    # or reordering whatever ops already exist.
    cube_translate_attr, cube_orient_attr = ensure_translate_orient_ops(cube_prim)
    bowl_translate_attr, bowl_orient_attr = ensure_translate_orient_ops(bowl_prim)

    cube_translate_attr.Set(Gf.Vec3d(*RESOLVED_CUBE_POSE.xyz_world_m))
    cube_orient_attr.Set(Gf.Quatf(*fixed_workspace.yaw_deg_to_quat_wxyz(RESOLVED_CUBE_POSE.yaw_world_deg)))
    bowl_translate_attr.Set(Gf.Vec3d(*RESOLVED_BOWL_POSE.xyz_world_m))
    bowl_orient_attr.Set(Gf.Quatf(*fixed_workspace.yaw_deg_to_quat_wxyz(RESOLVED_BOWL_POSE.yaw_world_deg)))

    # Consistency check only -- never auto-corrects a configured pose (see
    # utils/fixed_workspace.py's cube_table_contact_gap_m docstring).
    gap_m = fixed_workspace.cube_table_contact_gap_m(WORKSPACE_LAYOUT, RESOLVED_CUBE_POSE.xyz_world_m[2])
    if abs(gap_m) > TABLE_CONTACT_WARN_TOLERANCE_M:
        print(
            f"[WARN]: Cube world Z ({RESOLVED_CUBE_POSE.xyz_world_m[2]:.4f}m) is {gap_m * 1000.0:+.1f}mm "
            f"off the table surface (table_surface_z_world_m={WORKSPACE_LAYOUT.table_surface_z_world_m:.4f} "
            f"+ cube.size_m[2]/2={WORKSPACE_LAYOUT.cube.size_m[2] / 2.0:.4f}) -- not auto-corrected. "
            "Re-measure cube.xyz_base_m, or set cube.rest_on_table=true, if this isn't intentional."
        )
    robot_translate, robot_orient = ensure_translate_orient_ops(robot_prim)
    robot_translate.Set(Gf.Vec3d(*WORKSPACE_LAYOUT.robot_world.xyz_m))
    rotation = fixed_workspace.yaw_deg_to_quat_wxyz(WORKSPACE_LAYOUT.robot_world.yaw_deg)
    robot_orient.Set(Gf.Quatd(*rotation) if robot_orient.GetTypeName() == "quatd" else Gf.Quatf(*rotation))

    print(f"[LAYOUT] frame={WORKSPACE_LAYOUT.frame}")
    print(f"[LAYOUT] source={WORKSPACE_LAYOUT.source}")
    print(
        f"[LAYOUT] robot_world_xyz={_fmt_xyz(WORKSPACE_LAYOUT.robot_world.xyz_m)} "
        f"yaw_deg={WORKSPACE_LAYOUT.robot_world.yaw_deg:.2f}"
    )
    cube_base_str = _fmt_xyz(RESOLVED_CUBE_POSE.xyz_base_m) if RESOLVED_CUBE_POSE.xyz_base_m else "N/A (--cube_pos world override)"
    print(f"[LAYOUT] cube_base_xyz={cube_base_str}")
    print(f"[LAYOUT] cube_world_xyz={_fmt_xyz(RESOLVED_CUBE_POSE.xyz_world_m)}")
    print(f"[LAYOUT] cube_yaw_deg={RESOLVED_CUBE_POSE.yaw_world_deg:.2f} (source={RESOLVED_CUBE_POSE.yaw_source})")
    bowl_base_str = _fmt_xyz(RESOLVED_BOWL_POSE.xyz_base_m) if RESOLVED_BOWL_POSE.xyz_base_m else "N/A (--bowl_pos world override)"
    print(f"[LAYOUT] bowl_base_xyz={bowl_base_str}")
    print(f"[LAYOUT] bowl_world_xyz={_fmt_xyz(RESOLVED_BOWL_POSE.xyz_world_m)}")
    print(f"[LAYOUT] bowl_yaw_deg={RESOLVED_BOWL_POSE.yaw_world_deg:.2f} (source={RESOLVED_BOWL_POSE.yaw_source})")
    print(f"[LAYOUT] cube_dynamic=True; volume_deformable={soft_cube}; hard_rubber={is_rubber_cube(cube_prim)}")
    print("[LAYOUT] bowl_static=True")

    # Captured after the pose above is applied and before the timeline
    # plays, so this is each prim's *configured* starting pose (translate
    # and orient both) -- restored, along with zeroed cube velocity, on 'R'.
    cube_orig_pose = snapshot_xform_ops(cube_prim)
    bowl_orig_pose = snapshot_xform_ops(bowl_prim)

    # See keyboard_agent_raw_isaacsim.py -- root_joint's body0 binds to the
    # physics scene's literal origin, not the ancestor Xform's real position.
    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    if not root_joint_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    body1_path = root_joint_prim.GetRelationship("physics:body1").GetTargets()[0]
    body_world = UsdGeom.Xformable(stage.GetPrimAtPath(body1_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    world_anchor = body_world.Transform(Gf.Vec3d(root_joint_prim.GetAttribute("physics:localPos1").Get()))
    root_joint_prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(world_anchor))
    root_joint_prim.GetAttribute("physics:localRot0").Set(Gf.Quatf(body_world.ExtractRotationQuat()*Gf.Quatd(local_rot1)))

    # Apply the tuned actuator gains and cache each joint's limits/attributes.
    joints = {}
    for joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Expected joint prim not found: {joint_name}")

        if not (authored_contact and joint_name == "Jaw"):
            joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
            joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
            joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))

        target_attr = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        joints[joint_name] = {
            "target_attr": target_attr,
            "lower": joint_prim.GetAttribute("physics:lowerLimit").Get(),
            "upper": joint_prim.GetAttribute("physics:upperLimit").Get(),
        }

    timeline = omni.timeline.get_timeline_interface()
    if not args_cli.start_paused or SIM_TEST:
        timeline.play()
    simulation_app.update()

    keyboard_control = DualTeleopKeys()
    gate = DualActionGate(
        allow_gripper_start_mismatch=True, startup_mode=args_cli.startup_mode,
        timeout_s=args_cli.control_timeout_s, max_step_s=min(1/args_cli.control_hz, 1/30),
    ) if args_cli.start_paused and not SIM_TEST else None
    camera_preview = None
    if args_cli.show_cameras and not SIM_TEST:
        from sim_to_real_so101.utils.dual_camera_preview import DualCameraPreview
        camera_preview = DualCameraPreview(args_cli.camera_indices)
    if args_cli.scene_profile == "codex" and not args_cli.headless:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = "/World/Cameras/DemoCamera"
    if gate:
        print(f"[CONTROL]: Short loop delays continue with a {gate.max_step_s*1000:.1f}ms command-step cap; gaps over {gate.timeout_s:.2f}s pause until F8.", flush=True)
        if gate.startup_mode == "ramp":
            print("[PAUSED]: F8 starts BOTH arms with a 10-unit/s command ramp from the follower pose. Hold the leader still during convergence. F9 pauses; Esc exits.", flush=True)
        else:
            print("[PAUSED]: F8 checks the five arm joints and runs BOTH arms; gripper ramps from its current opening. F9 pauses; Esc exits.", flush=True)

    print_simulator_version_banner()
    print(f"[INFO]: {'No-hardware physics test' if SIM_TEST else 'Leader connected: '+args_cli.port}")
    print(f"[INFO]: HF_LEROBOT_CALIBRATION={os.environ['HF_LEROBOT_CALIBRATION']}")
    print("[INFO]: Driving joints from the leader arm. Ctrl+C or close the window to stop.")
    print("[INFO]: Click 'R' to reset the cube/bowl to their original positions")

    # Phase 2: measure, don't assume, the cost of a blocking follower serial
    # write on Kit's clock. Accumulated and flushed as a periodic summary
    # (not per-tick, to avoid log spam) so a --follower_port run's tick time
    # can be compared against a --follower_port-unset run.
    tick_count = 0
    tick_time_total = 0.0
    send_time_total = 0.0
    send_count = 0
    last_stats_print = time.perf_counter()
    test_start = time.perf_counter()
    last_preview = 0.
    last_delay_warning = -math.inf
    last_render_s = 0.0
    last_follower_send_s = 0.0
    max_tick_s = 0.0
    max_render_s = 0.0

    try:
        while simulation_app.is_running():
            tick_start = time.perf_counter()
            if SIM_TEST:
                simulation_app.update()
                if time.perf_counter()-test_start >= args_cli.sim_test_seconds:
                    cube_world = UsdGeom.Xformable(cube_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    if soft_cube:
                        skin_bounds = soft_cube_world_bounds(stage)
                        cube_world = (skin_bounds.GetMin()+skin_bounds.GetMax())/2
                    base_world = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PRIM_PATH+"/base")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    expected_z = WORKSPACE_LAYOUT.table_surface_z_world_m+WORKSPACE_LAYOUT.cube.size_m[2]/2
                    if abs(cube_world[2]-expected_z) > .01 or (base_world-Gf.Vec3d(*WORKSPACE_LAYOUT.robot_world.xyz_m)).GetLength() > .01:
                        raise RuntimeError(f"Physics smoke check failed: cube={cube_world}, base={base_world}")
                    print(f"[PASS]: Physics settling: cube={cube_world}, fixed base={base_world}; no hardware opened", flush=True)
                    break
                continue
            if keyboard_control.exit_requested:
                break
            if gate and keyboard_control.pause_requested:
                gate.pause()
                timeline.pause()
                keyboard_control.pause_requested = False
                keyboard_control.arm_requested = False
                print("[PAUSED]: follower holds its last target; physics paused", flush=True)
            if gate and gate.active and not timeline.is_playing():
                gate.pause("Timeline paused/stopped; press F8 to recheck the start pose")
                print(f"[PAUSED]: {gate.reason}", flush=True)
            if camera_preview and tick_start-last_preview > .1:
                camera_preview.update()
                last_preview = tick_start

            if keyboard_control.reset_world:
                if gate:
                    gate.pause("Scene reset; reset the real cube too before F8")
                    timeline.pause()
                if soft_cube:
                    # Stop/reinitialize physics to clear all FEM node velocities,
                    # rather than accidentally adding a rigid-body API on reset.
                    timeline.stop()
                    simulation_app.update()
                    restore_soft_cube_points(stage)
                restore_prim_pose(cube_prim, cube_orig_pose, zero_velocity=not soft_cube)
                restore_prim_pose(bowl_prim, bowl_orig_pose)
                keyboard_control.reset_world = False

            leader_read_start = time.perf_counter()
            try:
                real_action = robot_iface.robot.get_action()
            except Exception as exc:
                raise RuntimeError(f"Lost connection to leader on {args_cli.port}: {exc}") from exc
            leader_read_s = time.perf_counter()-leader_read_start

            if gate:
                if keyboard_control.arm_requested:
                    keyboard_control.arm_requested = False
                    try:
                        observed = follower_iface.robot.get_observation() if follower_connected else real_action
                        gate.arm(real_action, observed, time.perf_counter())
                        if gate.startup_ramping:
                            print(f"[STARTUP]: Ramping all joint commands from the measured follower pose at {gate.startup_speed:.1f} normalized units/s. Hold the leader still; F9 pauses.", flush=True)
                        if abs(real_action["gripper.pos"]-observed["gripper.pos"]) > gate.max_start_difference:
                            print(
                                f"[GRIPPER]: Starting at follower={observed['gripper.pos']:.1f}; "
                                f"tracking leader={real_action['gripper.pos']:.1f} at at most "
                                f"{gate.startup_speed if gate.startup_ramping else gate.speed:.1f} normalized units/s (no start-pose rejection).",
                                flush=True,
                            )
                        print("[RUNNING]: same limited command goes to real follower and simulated arm", flush=True)
                    except (ValueError, KeyError, RuntimeError) as exc:
                        gate.pause(str(exc))
                        print(f"[PAUSED]: {exc}", flush=True)
                was_active = gate.active
                was_ramping = gate.startup_ramping
                real_action = gate.next_action(real_action, time.perf_counter())
                if was_ramping and gate.active and not gate.startup_ramping:
                    print("[RUNNING]: Arm command ramp converged; normal teleoperation rate enabled.", flush=True)
                if real_action is None:
                    if was_active:
                        print(f"[PAUSED]: {gate.reason}", flush=True)
                    timeline.pause()
                    simulation_app.update()
                    time.sleep(max(0., 1/args_cli.control_hz-(time.perf_counter()-tick_start)))
                    continue
                if gate.last_gap_s > .5 and time.perf_counter()-last_delay_warning >= STATS_PRINT_INTERVAL_S:
                    last_delay_warning = time.perf_counter()
                    print(
                        f"[DELAY]: Loop gap {gate.last_gap_s:.3f}s; continuing with a capped command step, no catch-up jump. "
                        f"Previous render={last_render_s*1000:.1f}ms, current leader read={leader_read_s*1000:.1f}ms, "
                        f"previous follower send={last_follower_send_s*1000:.1f}ms.", flush=True,
                    )

            # Use the follower's returned (possibly gripper-limited) command as
            # the sim input too, so both receive the same command in this tick.
            if gate and follower_connected:
                send_start = time.perf_counter()
                try:
                    real_action = follower_iface.robot.send_action(real_action)
                    # Base the next slew step on the command actually accepted,
                    # including LeRobot's gripper stall relief.
                    gate.last_action = dict(real_action)
                except Exception as exc:
                    gate.pause(f"Follower communication failed: {exc}")
                    timeline.pause()
                    raise RuntimeError(gate.reason) from exc
                last_follower_send_s = time.perf_counter() - send_start
                send_time_total += last_follower_send_s
                send_count += 1

            mapped_deg = robot_iface.mapped_degrees(real_action)

            # wrist_roll's alignment-corrected target, captured for the
            # periodic debug printout below -- every other joint is set from
            # `mapped_deg` unmodified, exactly as before this fix.
            wrist_roll_target = None
            for joint_name, target_deg in zip(JOINT_ORDER, mapped_deg):
                info = joints[joint_name]
                if joint_name == "Wrist_Roll":
                    wrist_roll_target = WRIST_ROLL_ALIGNMENT.apply(target_deg, info["lower"], info["upper"])
                    clamped = wrist_roll_target.clamped_deg
                else:
                    clamped = max(info["lower"], min(info["upper"], target_deg))
                info["target_attr"].Set(clamped)

            # Mirror the same leader reading to the real follower, same tick,
            # unmodified raw dict -- no unit conversion needed, real_action is
            # already in the native lerobot key space a follower's
            # send_action() expects. See dual-teleop-sim-and-follower-plan.md
            # Phase 1 (why no conversion) and Phase 2 (why the try/except and
            # the timing measurement below).
            if gate is None and follower_connected and not follower_send_failed:
                send_start = time.perf_counter()
                try:
                    follower_iface.robot.send_action(real_action)
                except Exception as exc:
                    follower_send_failed = True
                    print(
                        f"[ERROR]: Lost connection to the follower arm, no longer mirroring to it "
                        f"for the rest of this run (sim continues normally): {exc}"
                    )
                else:
                    send_time_total += time.perf_counter() - send_start
                    send_count += 1

            if gate and gate.active:
                timeline.play()
            render_start = time.perf_counter()
            simulation_app.update()
            last_render_s = time.perf_counter()-render_start
            max_render_s = max(max_render_s, last_render_s)
            time.sleep(max(0., 1/args_cli.control_hz-(time.perf_counter()-tick_start)))

            tick_count += 1
            tick_elapsed_s = time.perf_counter() - tick_start
            tick_time_total += tick_elapsed_s
            max_tick_s = max(max_tick_s, tick_elapsed_s)
            now = time.perf_counter()
            if now - last_stats_print >= STATS_PRINT_INTERVAL_S:
                avg_tick_ms = (tick_time_total / tick_count) * 1000.0
                if send_count:
                    avg_send_ms = (send_time_total / send_count) * 1000.0
                    print(
                        f"[INFO]: avg tick={avg_tick_ms:.2f} ms over {tick_count} ticks "
                        f"(avg follower send_action={avg_send_ms:.2f} ms over {send_count} sends)"
                    )
                else:
                    print(f"[INFO]: avg tick={avg_tick_ms:.2f} ms over {tick_count} ticks")
                print(f"[INFO]: maximum tick={max_tick_s*1000:.1f} ms; maximum render={max_render_s*1000:.1f} ms", flush=True)
                # Diagnostic readout for the wrist_roll real-vs-sim alignment
                # fix (2026-08-24) -- every stage of the pipeline, so a
                # visual mismatch can be traced to a specific number while
                # watching the viewport: leader's raw lerobot reading (-100..
                # 100, NOT degrees despite the name), the SO101_USD_MAPPING-
                # scaled degree value, the configured direction/offset
                # correction (WRIST_ROLL_ALIGNMENT above) and what it
                # produced, and finally the clamped value actually written to
                # the sim joint drive this tick.
                _wr_raw = real_action["wrist_roll.pos"]
                wr = wrist_roll_target
                print(
                    f"[INFO]: wrist_roll -- leader raw={_wr_raw:+7.2f}  scaled={wr.scaled_deg:+7.2f} deg  "
                    f"direction={wr.direction_sign:+.1f}  offset={wr.zero_offset_deg:+7.2f} deg  "
                    f"unclamped={wr.unclamped_deg:+7.2f} deg  sim target(clamped)={wr.clamped_deg:+7.2f} deg"
                )
                tick_count = 0
                tick_time_total = 0.0
                send_time_total = 0.0
                send_count = 0
                last_stats_print = now
                max_tick_s = max_render_s = 0.0
    finally:
        if camera_preview:
            camera_preview.close()
        keyboard_control.cleanup()
        if connected:
            try:
                robot_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the leader arm: {exc}")
        if follower_connected:
            try:
                follower_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the follower arm: {exc}")
        timeline.stop()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except KeyboardInterrupt:
        print("[STOP]: Interrupted; disconnecting both arms.", flush=True)
        exit_code = 130
    except Exception as exc:
        print(f"[ERROR]: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        # Also clean up failures during scene setup, before the loop's finally.
        for interface in reversed(_CONNECTED_INTERFACES):
            robot = getattr(interface, "robot", None)
            bus = getattr(robot, "bus", None)
            if bus is not None and bus.is_connected:
                try:
                    bus.disconnect()
                except Exception as exc:
                    print(f"[WARN]: Final serial cleanup failed: {exc}")
        # Kit's fast shutdown can exit the process itself, bypassing sys.exit.
        simulation_app.close(exit_code=exit_code)
    sys.exit(exit_code)
