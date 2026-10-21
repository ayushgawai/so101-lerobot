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
"""Compare the REAL SO-101 follower's joint angles against the SIMULATED one.

Both followers are driven from the same physical leader arm, on the same
tick, through the same conversion -- exactly as
leader_arm_teleop_raw_isaacsim.py does it -- and then both are *read back*
and put in the same units, so the printed difference is real-vs-sim
tracking error and nothing else.

The pipeline this instruments, per tick, for the --joint focus joint
(default wrist_roll):

    SO-101 leader reading  (robot.get_action())
            |                 raw STS3215 counts, already through
            |                 lerobot's homing_offset/range_min/range_max
            v
    LeRobot calibration / normalization
            |                 `<joint>.pos`, MotorNormMode.RANGE_M100_100,
            |                 i.e. -100..100 -- NOT degrees
            v
    conversion / mapping  (LeRobotSO101Interface.get_mapped_actions_vectorized)
            |                 -100..100 -> 0..1 -> SO101_USD_MAPPING's
            |                 per-joint USD range (wrist_roll: -160..160 deg)
            v
    degrees -> radians
            |                 get_mapped_actions_vectorized already returns
            |                 radians; this script converts back x180/pi
            |                 because the raw USD drives are authored in
            |                 DEGREES (see the teleop script's point 2)
            v
    Isaac Sim wrist_roll target  (drive:angular:physics:targetPosition, deg)

and then, which is the part the teleop script does not do, reads the two
followers back:

  * sim measured -- the articulation's actual DOF position from the PhysX
    tensor API (radians), not the target. Target != position: the drive is
    a spring, so gravity, inertia and contact leave a steady-state error.
  * real measured -- SO101Follower.get_observation()'s `<joint>.pos`, pushed
    through the SAME get_mapped_actions_vectorized() mapping so it lands in
    USD/sim space. That shared mapping is what makes the two numbers
    subtractable at all; comparing lerobot's -100..100 against sim radians
    directly is meaningless.

Three gaps are reported separately, because they have different causes:
  real - cmd   the real arm's own servo tracking error (load, friction, gear
               backlash, STS3215 P-gain).
  sim  - cmd   the sim drive's tracking error (JOINT_GAINS stiffness/damping/
               maxForce, gravity, contact).
  real - sim   the sim-to-real gap this script exists to measure. A constant
               offset here points at calibration (see the wrist_roll note
               below); a lag that grows with speed points at the gains.

A per-joint summary (mean signed bias, mean |error|, max |error|) prints on
exit -- the bias column is the one to read for calibration problems, since
sign-cancelling noise averages out of it but a real offset does not.

wrist_roll is the default --joint for the same reason it is in
read_follower_wrist_roll.py: calibration/robots/so_follower/my_so_arm.json
has range_min=4095 range_max=8190 for it, above a 12-bit STS3215's 0..4095,
with a `.drifted_8190` backup sitting next to the file. If the real-vs-sim
bias on this joint is large and constant, suspect that calibration before
suspecting the sim.

Scene fidelity, stated plainly: this script applies the teleop script's
root_joint mount patch and its JOINT_GAINS verbatim (without them the
articulation is unconstrained and the DOF readings blow up to ~1e10 -- this
is measured, not theoretical), so the tracking numbers transfer to a teleop
run. It deliberately does NOT apply that script's cube RigidBodyAPI/mass/
friction patches: the cube stays as authored (static, no rigid body), so
nothing is grippable here. This measures free-space joint tracking. Joint
tracking *under grasp load* is a different measurement -- use the teleop
script, or test_grasp_dynamics.py, for that.

Needs a Python with torch + the vendored `lerobot` fork -- on this machine
that means Isaac Sim's python, not the repo's usdenv:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\compare_real_vs_sim_joints.py --port COM4 --follower_port COM3
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\compare_real_vs_sim_joints.py --port COM4 --follower_port COM3 --joint elbow_flex --csv out.csv
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\compare_real_vs_sim_joints.py --port COM4 --no_follower
"""
import argparse
import csv
import os
import sys
import time

# Match the other scripts: make sim_to_real_so101 importable without an
# editable pip install into whatever Python this is run with.
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

# Pure config/math (json/math/dataclasses only) -- importable before Kit
# boots, unlike the utils imported after SimulationApp() below.
from sim_to_real_so101.utils import fixed_workspace  # noqa: E402

parser = argparse.ArgumentParser(
    description="Compare real vs simulated SO-101 follower joint angles, both driven from one leader arm."
)
parser.add_argument("--headless", action="store_true", default=False, help="Run without a viewport window.")
parser.add_argument("--port", type=str, default=os.getenv("TELEOP_PORT", "COM4"), help="Leader arm serial port.")
parser.add_argument(
    "--robot_id",
    type=str,
    default=os.getenv("TELEOP_ID", "my_so_arm"),
    help="Leader arm calibration id (must match a file under calibration/teleoperators/so_leader/).",
)
parser.add_argument(
    "--follower_port",
    type=str,
    default=os.getenv("FOLLOWER_PORT", "COM3"),
    help="Real follower arm serial port (default COM3). Ignored when --no_follower is given.",
)
parser.add_argument(
    "--follower_robot_id",
    type=str,
    default="my_so_arm",
    help="Follower calibration id (must match a file under calibration/robots/so_follower/).",
)
parser.add_argument(
    "--no_follower",
    action="store_true",
    default=False,
    help="Don't connect a real follower. Degrades to a command-vs-sim comparison (the real_deg column "
    "reads n/a) -- useful for checking the sim drive's own tracking error with only a leader attached.",
)
parser.add_argument(
    "--calibration_dir",
    type=str,
    default=os.path.join(_REPO_ROOT_DIR, "calibration"),
    help="Root calibration directory (expects <this>/teleoperators/so_leader/<id>.json and "
    "<this>/robots/so_follower/<id>.json). Sets HF_LEROBOT_CALIBRATION -- must be set before "
    "`lerobot` is imported.",
)
parser.add_argument(
    "--joint",
    type=str,
    default="wrist_roll",
    help="Focus joint for the per-tick pipeline trace (default wrist_roll -- see the module docstring). "
    "Every joint is still compared in the table and the exit summary; this only picks which one gets "
    "the step-by-step breakdown. One of: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, "
    "wrist_roll, gripper.",
)
parser.add_argument(
    "--sample_hz",
    type=float,
    default=10.0,
    help="How often to read the real follower back and record a comparison row (default 10). This costs "
    "an extra serial round-trip per sample on top of the per-tick send_action, which is why it's "
    "decoupled from the sim tick rate.",
)
parser.add_argument(
    "--print_hz",
    type=float,
    default=2.0,
    help="How often to print a comparison block to the console (default 2). Samples between prints still "
    "feed the CSV and the exit summary.",
)
parser.add_argument(
    "--csv",
    type=str,
    default=None,
    help="Optional path to write one row per sample (t_s, and cmd/real/sim degrees for all six joints).",
)
parser.add_argument(
    "--duration",
    type=float,
    default=0.0,
    help="Stop after this many seconds and print the summary. 0 (default) = run until Ctrl+C or the "
    "window is closed.",
)
parser.add_argument(
    "--layout",
    type=str,
    default=None,
    help="Path to a fixed-workspace layout JSON. Only its robot_world mount pose is used here (the "
    "cube/bowl are irrelevant to joint tracking) -- but the mount patch itself is NOT optional, see "
    "the module docstring. Unset = the in-code legacy default layout.",
)
parser.add_argument(
    "--settle_steps",
    type=int,
    default=30,
    help="Sim steps to run after play() before the first reading, so the articulation settles and the "
    "PhysX tensor view is populated. Default 30.",
)
args_cli = parser.parse_args()

# Fail fast on bad ids/paths before paying for Kit's ~30s+ boot -- lerobot's
# own error for a missing calibration file only surfaces much later, from
# inside connect()'s serial handshake.
_LEADER_CALIBRATION_FILE = os.path.join(
    args_cli.calibration_dir, "teleoperators", "so_leader", f"{args_cli.robot_id}.json"
)
if not os.path.isfile(_LEADER_CALIBRATION_FILE):
    print(f"[ERROR]: Leader calibration file not found: {_LEADER_CALIBRATION_FILE}", file=sys.stderr)
    print("[ERROR]: Check --robot_id / --calibration_dir (or TELEOP_ID / TELEOP_PORT).", file=sys.stderr)
    sys.exit(1)

if not args_cli.no_follower:
    _FOLLOWER_CALIBRATION_FILE = os.path.join(
        args_cli.calibration_dir, "robots", "so_follower", f"{args_cli.follower_robot_id}.json"
    )
    if not os.path.isfile(_FOLLOWER_CALIBRATION_FILE):
        print(f"[ERROR]: Follower calibration file not found: {_FOLLOWER_CALIBRATION_FILE}", file=sys.stderr)
        print("[ERROR]: Check --follower_robot_id / --calibration_dir, or pass --no_follower.", file=sys.stderr)
        sys.exit(1)

if args_cli.sample_hz <= 0:
    print("[ERROR]: --sample_hz must be > 0.", file=sys.stderr)
    sys.exit(1)

try:
    WORKSPACE_LAYOUT = (
        fixed_workspace.load_layout(args_cli.layout)
        if args_cli.layout is not None
        else fixed_workspace.default_layout()
    )
except fixed_workspace.LayoutError as exc:
    print(f"[ERROR]: {exc}", file=sys.stderr)
    sys.exit(1)

# Must happen before any `lerobot` import (including transitively via
# sim_to_real_so101.utils.lerobot_interface) -- lerobot.utils.constants reads
# this at import time to compute HF_LEROBOT_CALIBRATION, which is what
# resolves the calibration JSONs checked above.
os.environ["HF_LEROBOT_CALIBRATION"] = args_cli.calibration_dir

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args_cli.headless})

"""Kit is booted -- pxr/omni/isaacsim.core are importable from here down."""

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402

from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface  # noqa: E402
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.join(_DEMO_DIR, "real-to-sim.usd")
ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"

# Verbatim from leader_arm_teleop_raw_isaacsim.py -- the whole point of this
# script is to characterize THAT script's sim, so these must not drift from
# it. See its JOINT_GAINS comment for why the Isaac-Lab-tuned values are used
# unconverted on the raw PhysicsDriveAPI attributes.
JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=30),
}
JOINT_ORDER = list(JOINT_GAINS.keys())

# The teleop script relies on a *positional* correspondence between
# SO101_JOINT_ORDER and JOINT_ORDER rather than a name map. That's fine for
# writing targets, but this script also has to read specific DOFs back out
# of the articulation by name, so the correspondence is made explicit here
# and then cross-checked against it at startup (see _resolve_dof_indices).
LEROBOT_TO_USD_JOINT = {
    "shoulder_pan": "Rotation",
    "shoulder_lift": "Pitch",
    "elbow_flex": "Elbow",
    "wrist_flex": "Wrist_Pitch",
    "wrist_roll": "Wrist_Roll",
    "gripper": "Jaw",
}

RAD_TO_DEG = 180.0 / 3.141592653589793
DEG_TO_RAD = 3.141592653589793 / 180.0

# Follower startup sync -- copied from leader_arm_teleop_raw_isaacsim.py.
# SO101FollowerConfig.max_relative_target is never set by
# LeRobotSO101Interface.make_cfg(), so nothing in lerobot limits the first
# send_action()'s jump; ramping over a fixed short duration is a mitigation,
# not a guarantee, and does not replace putting the physical leader near the
# follower's pose before connecting.
FOLLOWER_STARTUP_SYNC_DURATION_S = 1.5
FOLLOWER_STARTUP_SYNC_STEPS = 45


def _ramp_follower_to_leader_start(follower_robot, leader_action: dict) -> None:
    """Gradually move a just-connected follower from its current pose to the
    leader's, so the first real send_action() isn't a snap-to-target."""
    try:
        obs = follower_robot.get_observation()
    except Exception as exc:
        print(f"[WARN]: Could not read the follower's start pose ({exc}) -- skipping the startup ramp.")
        return

    start = {k: v for k, v in obs.items() if k in leader_action}
    if not start:
        print("[WARN]: Follower observation had no joints in common with the leader -- skipping the ramp.")
        return

    largest = max(abs(leader_action[k] - start[k]) for k in start)
    print(
        f"[INFO]: Follower startup sync: ramping over {FOLLOWER_STARTUP_SYNC_DURATION_S:.1f}s "
        f"(largest joint move: {largest:.1f} normalized units)."
    )

    step_dt = FOLLOWER_STARTUP_SYNC_DURATION_S / FOLLOWER_STARTUP_SYNC_STEPS
    for step in range(1, FOLLOWER_STARTUP_SYNC_STEPS + 1):
        alpha = step / FOLLOWER_STARTUP_SYNC_STEPS
        interpolated = {k: start[k] + alpha * (leader_action[k] - start[k]) for k in start}
        try:
            follower_robot.send_action(interpolated)
        except Exception as exc:
            print(f"[WARN]: Follower startup sync aborted mid-ramp ({exc}) -- entering normal comparison now.")
            return
        time.sleep(step_dt)
    print("[INFO]: Follower startup sync complete.")


def _resolve_dof_indices(articulation) -> list:
    """Map SO101_JOINT_ORDER onto articulation DOF indices *by name*, then
    check that result against the positional assumption the teleop script
    makes. A silent mismatch here would produce a plausible-looking but
    wrong per-joint comparison, which is worse than a crash."""
    dof_names = list(articulation.dof_names)
    indices = []
    for slot, lerobot_key in enumerate(LeRobotSO101Interface.SO101_JOINT_ORDER):
        usd_name = LEROBOT_TO_USD_JOINT[lerobot_key.split(".")[0]]
        if usd_name not in dof_names:
            raise RuntimeError(
                f"DOF {usd_name!r} (for {lerobot_key}) not found in the articulation. Got: {dof_names}"
            )
        index = dof_names.index(usd_name)
        if index != slot:
            print(
                f"[WARN]: {usd_name} is DOF #{index} but the teleop script's positional JOINT_ORDER "
                f"assumes #{slot} -- this script reads by name and stays correct, but that script's "
                "joint targets would be crossed. Investigate before trusting a teleop run."
            )
        indices.append(index)
    return indices


def _mapped_deg(iface, pos_dict: dict) -> list:
    """lerobot `<joint>.pos` dict -> USD/sim degrees, one entry per
    SO101_JOINT_ORDER slot. This is the single shared mapping applied to the
    leader command, and to the real follower's own reading, which is what
    makes them comparable against the sim's DOF positions."""
    tensor = iface.get_raw_actions_tensor(pos_dict)
    return (iface.get_mapped_actions_vectorized(tensor) * RAD_TO_DEG).tolist()


def main():
    joint = args_cli.joint
    joint_key = f"{joint}.pos"
    if joint_key not in LeRobotSO101Interface.SO101_JOINT_ORDER:
        valid = ", ".join(j.split(".")[0] for j in LeRobotSO101Interface.SO101_JOINT_ORDER)
        print(f"[ERROR]: Unknown --joint {joint!r}. Valid joints: {valid}", file=sys.stderr)
        sys.exit(1)
    focus_slot = LeRobotSO101Interface.SO101_JOINT_ORDER.index(joint_key)
    joint_names = [k.split(".")[0] for k in LeRobotSO101Interface.SO101_JOINT_ORDER]

    # Arm connections happen BEFORE any scene/physics/rendering setup below.
    # Root-caused in leader_arm_teleop_raw_isaacsim.py (2026-08-24, see its
    # main() comment): follower.connect()'s burst of ~42 rapid serial writes
    # silently kills the whole Kit process with no Python traceback when it
    # runs while the USD stage is loaded and physics is playing -- reproduced
    # identically here (this script is even busier at connect time than that
    # one was: stage loaded, root_joint patched, physics playing, AND 30
    # articulation settle steps already ticked). Connecting first, while Kit
    # is idle, avoids the contention.
    #
    # Leader -- the single command source for both followers.
    leader_iface = LeRobotSO101Interface(
        device="cpu", port=args_cli.port, id=args_cli.robot_id, cameras={}, fps=30, kind="leader"
    )
    try:
        leader_iface.init_device()
        leader_iface.connect()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to connect to the leader arm on port={args_cli.port} id={args_cli.robot_id}: {exc}. "
            "Check the arm is powered, plugged into the given --port, and not already held open by "
            "another process."
        ) from exc
    leader_connected = True
    print(f"[INFO]: Leader arm connected: port={args_cli.port} id={args_cli.robot_id}")

    follower_iface = None
    follower_connected = False
    follower_failed = False
    if not args_cli.no_follower:
        follower_iface = LeRobotSO101Interface(
            device="cpu",
            port=args_cli.follower_port,
            id=args_cli.follower_robot_id,
            cameras={},
            fps=30,
            kind="follower",
        )
        try:
            follower_iface.init_device()
            follower_iface.connect()
            follower_connected = True
        except Exception as exc:
            raise RuntimeError(
                f"Failed to connect to the follower arm on port={args_cli.follower_port} "
                f"id={args_cli.follower_robot_id}: {exc}. Check the arm is powered and plugged into the "
                "given --follower_port, or pass --no_follower to compare command-vs-sim only."
            ) from exc
        print(f"[INFO]: Follower arm connected: port={args_cli.follower_port} id={args_cli.follower_robot_id}")
        try:
            startup_action = leader_iface.robot.get_action()
        except Exception as exc:
            print(f"[WARN]: Could not read the leader for the follower startup sync ({exc}) -- skipping.")
        else:
            _ramp_follower_to_leader_start(follower_iface.robot, startup_action)
    else:
        print("[INFO]: --no_follower: comparing the leader command against the sim only.")

    usd_context = omni.usd.get_context()
    usd_context.open_stage(REAL_TO_SIM_USD)
    simulation_app.update()
    stage = usd_context.get_stage()

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}")

    # Not optional -- root_joint's body0 binds to the physics scene's literal
    # origin rather than the ancestor Xform's position, and leaving it
    # unpatched drives the DOF readings this script exists to print to ~1e10.
    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    if not root_joint_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    root_joint_prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*WORKSPACE_LAYOUT.robot_world.xyz_m))
    root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

    joints = {}
    for usd_joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{usd_joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Expected joint prim not found: {usd_joint_name}")
        joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
        joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
        joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))
        joints[usd_joint_name] = {
            "target_attr": joint_prim.GetAttribute("drive:angular:physics:targetPosition"),
            "lower": joint_prim.GetAttribute("physics:lowerLimit").Get(),
            "upper": joint_prim.GetAttribute("physics:upperLimit").Get(),
        }

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    articulation = Articulation(ROBOT_PRIM_PATH)
    for _ in range(max(1, args_cli.settle_steps)):
        simulation_app.update()
    if not articulation.valid or not articulation.is_physics_tensor_entity_valid():
        raise RuntimeError(
            f"Articulation at {ROBOT_PRIM_PATH} never became a valid PhysX tensor entity -- try a larger "
            "--settle_steps, and check the timeline is actually playing."
        )
    dof_indices = _resolve_dof_indices(articulation)
    print(f"[INFO]: articulation DOFs: {list(articulation.dof_names)}")

    csv_file = None
    csv_writer = None
    if args_cli.csv:
        csv_file = open(args_cli.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        header = ["t_s"]
        for name in joint_names:
            header += [f"{name}_cmd_deg", f"{name}_real_deg", f"{name}_sim_deg"]
        csv_writer.writerow(header)
        print(f"[INFO]: Writing samples to {args_cli.csv}")

    print(f"[INFO]: HF_LEROBOT_CALIBRATION={os.environ['HF_LEROBOT_CALIBRATION']}")
    print(
        f"[INFO]: focus joint = {joint} (USD DOF {LEROBOT_TO_USD_JOINT[joint]}, "
        f"sim range {LeRobotSO101Interface.SO101_USD_MAPPING[joint]['joint_min']}.."
        f"{LeRobotSO101Interface.SO101_USD_MAPPING[joint]['joint_max']} deg)"
    )
    print(f"[INFO]: sampling at {args_cli.sample_hz} Hz, printing at {args_cli.print_hz} Hz. Ctrl+C to stop.")

    # Per-joint accumulators for the exit summary. Signed sums are kept
    # separately from absolute ones because a constant calibration offset
    # shows up in the signed mean while sign-flipping tracking lag does not.
    stats = {name: {"n": 0, "sum_signed": 0.0, "sum_abs": 0.0, "max_abs": 0.0} for name in joint_names}
    clamp_warned = set()

    sample_period = 1.0 / args_cli.sample_hz
    print_period = 1.0 / args_cli.print_hz if args_cli.print_hz > 0 else float("inf")
    t0 = time.perf_counter()
    next_sample = t0
    next_print = t0

    try:
        while simulation_app.is_running():
            now = time.perf_counter()
            if args_cli.duration > 0 and (now - t0) >= args_cli.duration:
                print(f"[INFO]: --duration {args_cli.duration}s reached.")
                break

            # --- SO-101 leader reading -> LeRobot calibration/normalization
            try:
                leader_action = leader_iface.robot.get_action()
            except Exception as exc:
                print(f"[ERROR]: Lost connection to the leader arm: {exc}")
                break

            # --- conversion/mapping -> degrees (get_mapped_actions_vectorized
            # returns radians; x180/pi because the USD drives are in degrees)
            cmd_deg = _mapped_deg(leader_iface, leader_action)

            # --- degrees -> Isaac Sim joint drive targets
            for slot, (usd_joint, target_deg) in enumerate(zip(JOINT_ORDER, cmd_deg)):
                info = joints[usd_joint]
                clamped = max(info["lower"], min(info["upper"], target_deg))
                if abs(clamped - target_deg) > 1e-6 and usd_joint not in clamp_warned:
                    clamp_warned.add(usd_joint)
                    print(
                        f"[WARN]: {joint_names[slot]} command {target_deg:.2f} deg is outside the USD joint "
                        f"limit {info['lower']:.2f}..{info['upper']:.2f} and is being clamped -- the sim "
                        "cannot follow the real arm here, so treat this joint's gap as saturation, not "
                        "tracking error. (Warned once per joint.)"
                    )
                info["target_attr"].Set(clamped)

            # Mirror the same reading to the real follower, same tick, raw
            # dict -- leader_action is already in the key/unit space
            # send_action() expects, so no conversion belongs here.
            if follower_connected and not follower_failed:
                try:
                    follower_iface.robot.send_action(leader_action)
                except Exception as exc:
                    follower_failed = True
                    print(f"[ERROR]: Lost the follower arm, no longer commanding it (sim continues): {exc}")

            simulation_app.update()

            now = time.perf_counter()
            if now < next_sample:
                continue
            next_sample = now + sample_period

            # --- read both followers back, in the same units ---
            sim_rad = articulation.get_dof_positions().numpy()[0]
            sim_deg = [float(sim_rad[i]) * RAD_TO_DEG for i in dof_indices]

            real_deg = None
            if follower_connected and not follower_failed:
                try:
                    obs = follower_iface.robot.get_observation()
                except Exception as exc:
                    follower_failed = True
                    print(f"[ERROR]: Lost the follower arm while reading it back (sim continues): {exc}")
                else:
                    pos = {k: v for k, v in obs.items() if k.endswith(".pos")}
                    missing = [k for k in LeRobotSO101Interface.SO101_JOINT_ORDER if k not in pos]
                    if missing:
                        print(f"[ERROR]: Follower observation missing {missing} -- got {sorted(pos)}")
                    else:
                        real_deg = _mapped_deg(follower_iface, pos)

            t_s = now - t0
            if real_deg is not None:
                for slot, name in enumerate(joint_names):
                    err = real_deg[slot] - sim_deg[slot]
                    s = stats[name]
                    s["n"] += 1
                    s["sum_signed"] += err
                    s["sum_abs"] += abs(err)
                    s["max_abs"] = max(s["max_abs"], abs(err))

            if csv_writer is not None:
                row = [f"{t_s:.4f}"]
                for slot in range(len(joint_names)):
                    row += [
                        f"{cmd_deg[slot]:.4f}",
                        "" if real_deg is None else f"{real_deg[slot]:.4f}",
                        f"{sim_deg[slot]:.4f}",
                    ]
                csv_writer.writerow(row)

            if now < next_print:
                continue
            next_print = now + print_period

            # The focus joint's full pipeline, one line per stage.
            # Variable names below match the documented trace (wrist_roll shown
            # as the default --joint; any --joint substitutes into the same slots).
            wrist_roll_deg = cmd_deg[focus_slot]
            focus_info = joints[LEROBOT_TO_USD_JOINT[joint]]
            focus_target_deg = max(focus_info["lower"], min(focus_info["upper"], wrist_roll_deg))
            wrist_roll_rad = focus_target_deg * DEG_TO_RAD

            print(f"\n--- t={t_s:7.2f}s  focus joint: {joint} " + "-" * 34)
            print(f"leader {joint}:", leader_action[joint_key])
            print(f"converted {joint} deg:", wrist_roll_deg)
            print(f"isaac {joint} rad:", wrist_roll_rad)
            print(
                f"  sim  measured : {sim_deg[focus_slot]:+9.3f} deg  "
                f"({sim_deg[focus_slot] * DEG_TO_RAD:+.5f} rad)"
                f"   [sim  - cmd = {sim_deg[focus_slot] - wrist_roll_deg:+7.3f} deg]"
            )
            if real_deg is None:
                print("  real measured :       n/a  (no follower connected)")
            else:
                print(
                    f"  real measured : {real_deg[focus_slot]:+9.3f} deg  "
                    f"({real_deg[focus_slot] * DEG_TO_RAD:+.5f} rad)"
                    f"   [real - cmd = {real_deg[focus_slot] - wrist_roll_deg:+7.3f} deg]"
                )
                print(f"  real - sim    : {real_deg[focus_slot] - sim_deg[focus_slot]:+9.3f} deg")

            print(f"  {'joint':<14}{'cmd_deg':>10}{'real_deg':>10}{'sim_deg':>10}{'real-sim':>10}")
            for slot, name in enumerate(joint_names):
                real_cell = "n/a" if real_deg is None else f"{real_deg[slot]:+.3f}"
                gap_cell = "n/a" if real_deg is None else f"{real_deg[slot] - sim_deg[slot]:+.3f}"
                print(
                    f"  {name:<14}{cmd_deg[slot]:>+10.3f}{real_cell:>10}{sim_deg[slot]:>+10.3f}{gap_cell:>10}"
                )
    except KeyboardInterrupt:
        print("\n[INFO]: Stopped.")
    finally:
        if csv_file is not None:
            csv_file.close()
        if leader_connected:
            try:
                leader_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the leader arm: {exc}")
        if follower_connected:
            try:
                follower_iface.robot.disconnect()
            except Exception as exc:
                print(f"[WARN]: Failed to cleanly disconnect the follower arm: {exc}")
        timeline.stop()

        total = sum(s["n"] for s in stats.values())
        if total == 0:
            print("\n[INFO]: No real-vs-sim samples were collected -- nothing to summarize.")
        else:
            print("\n=== real vs sim, per joint (degrees in USD/sim space) " + "=" * 12)
            print(f"  {'joint':<14}{'n':>6}{'bias':>10}{'mean|err|':>11}{'max|err|':>10}")
            for name in joint_names:
                s = stats[name]
                if not s["n"]:
                    continue
                print(
                    f"  {name:<14}{s['n']:>6}{s['sum_signed'] / s['n']:>+10.3f}"
                    f"{s['sum_abs'] / s['n']:>11.3f}{s['max_abs']:>10.3f}"
                )
            print("  bias = mean(real - sim): a large constant value here points at calibration, not gains.")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
