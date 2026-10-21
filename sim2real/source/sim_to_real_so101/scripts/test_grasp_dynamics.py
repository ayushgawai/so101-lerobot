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
"""Standalone diagnostic: scripted reach/close/lift over the cube's actual
measured position, to test whether sim contact physics alone (friction +
jaw effort, no Step 3 kinematic latch) produces a genuine dynamic grasp.

Not part of the real->sim mirroring plan's core deliverables -- built to
answer a specific question raised mid-implementation: is the existing
friction/effort tuning (AWS_CUBE_*_FRICTION, GRIPPER_*_FRICTION,
JOINT_GAINS["Jaw"]) actually strong enough to hold and lift the cube via
real contact dynamics? The eval_sanity-act recorded episode couldn't answer
this -- its own gripper trajectory never descends within 8cm of table
height (confirmed both by live position tracking and independent forward-
kinematics recomputation), so it's the wrong episode to test grasp-height
physics against, on top of the already-known XY position mismatch.

A hand-rolled degree-space IK was tried first and abandoned: self-consistent
on paper (predicted its own solved angles to <1mm) but the sim, when
actually commanded to those angles, converged ~6cm away from the FK
prediction in Z -- a real bug isolated to that one-off custom chain, not to
the demo's actual real/sim mirroring (which never uses hand-rolled IK; it
drives joints from real recorded/live angles through
LeRobotSO101Interface's mapping, already verified extensively elsewhere in
this repo). This version sidesteps that bug rather than chasing it further:
it seeds from a REAL recorded raw joint state (eval_sanity-act frame 1614)
that this same mapping pipeline already proved puts the gripper at a known
position, then extrapolates in raw space -- no custom kinematics at all.

Run with the same Isaac Sim Python as the other scripts:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\test_grasp_dynamics.py
"""
import argparse
import os
import sys
import time

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Scripted reach/close/lift test of dynamic (non-latched) grasp physics.")
parser.add_argument("--headless", action="store_true", default=False)
args_cli = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args_cli.headless})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface  # noqa: E402
from sim_to_real_so101.utils.physics_material import apply_friction_material  # noqa: E402
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402
from sim_to_real_so101.utils.xform_prim_compat import make_xform_prim  # noqa: E402

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.join(_DEMO_DIR, "real-to-sim.usd")

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
AWS_CUBE_PRIM_PATH = "/World/AWSBuilderCube"
AWS_CUBE_MASS_KG = 0.05
AWS_CUBE_COLLISION_MESH_PATH = "/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo"
GRIPPER_COLLISION_PATH = "/World/SO_ARM101_USD/gripper/collisions"
JAW_COLLISION_PATH = "/World/SO_ARM101_USD/jaw/collisions"

# PVC cube + 3D-printed jaw/gripper -- matches PhysicsMaterialAPI on real-to-sim.usd.
AWS_CUBE_STATIC_FRICTION = 0.45
AWS_CUBE_DYNAMIC_FRICTION = 0.40
AWS_CUBE_RESTITUTION = 0.0
GRIPPER_STATIC_FRICTION = 0.40
GRIPPER_DYNAMIC_FRICTION = 0.35
GRIPPER_RESTITUTION = 0.0

ROBOT_POS = Gf.Vec3f(0.0, 0.3, 0.72)
REAL_CUBE_POS = Gf.Vec3d(0.0, 0.047, 0.7754)

JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=3),
}
JOINT_ORDER = list(JOINT_GAINS.keys())
RAD_TO_DEG = 180.0 / 3.141592653589793

# A hand-rolled degree-space IK (damped least squares against this repo's
# own FK derivation) was tried first and abandoned: it was internally
# self-consistent (predicted its own solved angles correctly) but those
# same angles, when actually commanded, converged the sim to a position
# ~6cm off in Z from the FK prediction -- a real, unresolved discrepancy in
# that one-off custom chain, isolated to that tool, not to the actual
# demo's real/sim mirroring (which drives joints from real recorded/live
# angles through LeRobotSO101Interface's mapping, not through any
# hand-rolled IK, and has been separately verified extensively).
#
# This version sidesteps that bug entirely by never using the custom IK:
# it seeds from frame 1614 of eval_sanity-act -- a REAL recorded raw state
# that this exact mapping pipeline already proved (via the replay script's
# own live position tracking) puts the gripper at a known, sensible world
# position (~0.86 in Z, the lowest that recording's gripper ever reached).
# From there it extrapolates shoulder_lift further in the same direction
# real motion was already headed at that point, entirely in raw ("-100..100")
# space through the same get_mapped_actions_vectorized() the working
# replay/teleop scripts use -- no custom kinematics involved at all.
REAL_SEED_RAW = {
    "shoulder_pan.pos": 5.0989013,
    "shoulder_lift.pos": 25.846153,
    "elbow_flex.pos": -2.7692308,
    "wrist_flex.pos": 58.76923,
    "wrist_roll.pos": -87.95605,
}
JAW_OPEN_RAW = 80.0
JAW_CLOSED_RAW = 0.0
LIFT_SHOULDER_LIFT_DELTA_RAW = -35.0  # raise after closing

# Two rounds of hand-extrapolated waypoints (single-joint, then coordinated
# 3-joint) both plateaued around jaw Z=0.83, still ~5.5cm above table --
# confirmed by direct measurement. Checked the FULL 50-episode training
# dataset's per-joint stats (meta/stats.json, so101-lerobot/hf_data) instead
# of extrapolating further blind: across every real successful grasp in
# that dataset, elbow_flex ranges down to -58.2 deg and wrist_flex up to
# 99.6 deg -- both well beyond anything tried so far (-35 and 78
# respectively). The seed pose was nowhere near representative of an actual
# grasp-depth pose; real grasps bend the elbow and point the wrist much
# further than this session's guesses did.
RAW_FULL_WAYPOINTS = [
    {"shoulder_lift.pos": 25.8, "elbow_flex.pos": -2.77, "wrist_flex.pos": 58.77},  # seed (real frame 1614)
    {"shoulder_lift.pos": 40.0, "elbow_flex.pos": -30.0, "wrist_flex.pos": 75.0},
    {"shoulder_lift.pos": 55.0, "elbow_flex.pos": -55.0, "wrist_flex.pos": 95.0},  # near real dataset extremes
]


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(REAL_TO_SIM_USD)
    simulation_app.update()
    stage = usd_context.get_stage()

    cube_prim = stage.GetPrimAtPath(AWS_CUBE_PRIM_PATH)
    if not cube_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_PRIM_PATH}")
    if not cube_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    if not cube_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(cube_prim).CreateMassAttr(AWS_CUBE_MASS_KG)

    cube_collision_mesh = stage.GetPrimAtPath(AWS_CUBE_COLLISION_MESH_PATH)
    apply_friction_material(cube_collision_mesh, AWS_CUBE_STATIC_FRICTION, AWS_CUBE_DYNAMIC_FRICTION, AWS_CUBE_RESTITUTION)
    for collision_path in (GRIPPER_COLLISION_PATH, JAW_COLLISION_PATH):
        collision_prim = stage.GetPrimAtPath(collision_path)
        apply_friction_material(collision_prim, GRIPPER_STATIC_FRICTION, GRIPPER_DYNAMIC_FRICTION, GRIPPER_RESTITUTION)

    cube_prim.GetAttribute("xformOp:translate").Set(REAL_CUBE_POS)

    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    root_joint_prim.GetAttribute("physics:localPos0").Set(ROBOT_POS)
    root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

    joints = {}
    for joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
        joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
        joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))
        joints[joint_name] = {
            "target_attr": joint_prim.GetAttribute("drive:angular:physics:targetPosition"),
            "lower": joint_prim.GetAttribute("physics:lowerLimit").Get(),
            "upper": joint_prim.GetAttribute("physics:upperLimit").Get(),
        }

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    cube_z_attr = cube_prim.GetAttribute("xformOp:translate")
    gripper_view = make_xform_prim(f"{ROBOT_PRIM_PATH}/gripper")
    jaw_view = make_xform_prim(f"{ROBOT_PRIM_PATH}/jaw")

    # Same conversion the working replay/teleop scripts use -- never
    # connected, only used for its stateless mapping math.
    conv_iface = LeRobotSO101Interface(device="cpu", port=None, id="unused", cameras={}, fps=30, kind="leader")

    def apply_raw(overrides: dict, jaw_raw: float):
        raw_state = dict(REAL_SEED_RAW, **overrides, **{"gripper.pos": jaw_raw})
        raw_tensor = conv_iface.get_raw_actions_tensor(raw_state)
        mapped_deg = conv_iface.get_mapped_actions_vectorized(raw_tensor) * RAD_TO_DEG
        for joint_name, target_deg in zip(JOINT_ORDER, mapped_deg.tolist()):
            info = joints[joint_name]
            clamped = max(info["lower"], min(info["upper"], target_deg))
            info["target_attr"].Set(clamped)

    def log(label: str, i: int):
        if i % 5 == 0:
            cz = cube_z_attr.Get()
            gp, _ = gripper_view.get_world_poses()
            gx, gy, gz = gp[0].tolist()
            print(
                f"[GRASP_TEST]: {label} t={i/30:.2f}s cube=({cz[0]:.4f},{cz[1]:.4f},{cz[2]:.4f}) "
                f"gripper=({gx:.4f},{gy:.4f},{gz:.4f})"
            )

    def ramp_to(target: dict, jaw_raw: float, seconds: float, label: str, start: dict, start_jaw: float):
        """Move in small per-tick increments (mimics how every working script in this
        repo drives joints -- from a smoothly-changing real trajectory, never a snap
        step). A direct jump caused violent transient swings that knocked the cube
        clean off its measured position before settling (found empirically: the cube
        ended up 20cm away after a snap-to-target sequence, despite Z looking flat --
        only Z was logged during transit, X/Y drift went unnoticed until the final
        position check)."""
        steps = max(1, int(seconds * 30))
        keys = target.keys()
        for i in range(steps):
            t = (i + 1) / steps
            overrides = {k: start[k] + (target[k] - start[k]) * t for k in keys}
            apply_raw(overrides, start_jaw + (jaw_raw - start_jaw) * t)
            simulation_app.update()
            log(label, i)
            time.sleep(1.0 / 30.0)

    zero_start = {"shoulder_lift.pos": 0.0, "elbow_flex.pos": 0.0, "wrist_flex.pos": 0.0}
    print("[GRASP_TEST]: ramping smoothly from rest to seed pose")
    ramp_to(RAW_FULL_WAYPOINTS[0], JAW_OPEN_RAW, 4.0, "ramp-to-seed", zero_start, JAW_OPEN_RAW)
    for _ in range(30):
        simulation_app.update()
        time.sleep(1.0 / 30.0)

    print("[GRASP_TEST]: descending smoothly through waypoints")
    prev_wp = RAW_FULL_WAYPOINTS[0]
    for i, wp in enumerate(RAW_FULL_WAYPOINTS[1:], start=1):
        ramp_to(wp, JAW_OPEN_RAW, 1.5, f"descend[{i}]", prev_wp, JAW_OPEN_RAW)
        prev_wp = wp

    print("[GRASP_TEST]: closing jaw")
    ramp_to(RAW_FULL_WAYPOINTS[-1], JAW_CLOSED_RAW, 2.0, "close", prev_wp, JAW_OPEN_RAW)
    for i in range(60):  # hold closed, let contact settle
        simulation_app.update()
        log("hold", i)
        time.sleep(1.0 / 30.0)

    jaw_pos_np, _ = jaw_view.get_world_poses()
    gripper_pos_np, _ = gripper_view.get_world_poses()
    cube_pos = cube_z_attr.Get()
    jaw_pos = jaw_pos_np[0].tolist()
    gripper_pos = gripper_pos_np[0].tolist()
    print(f"[GRASP_TEST]: live gripper pos={gripper_pos}")
    print(f"[GRASP_TEST]: live jaw pos={jaw_pos}")
    print(f"[GRASP_TEST]: live cube pos={list(cube_pos)}")
    dist_jaw_cube = sum((a - b) ** 2 for a, b in zip(jaw_pos, cube_pos)) ** 0.5
    print(f"[GRASP_TEST]: jaw-to-cube distance={dist_jaw_cube:.4f} m")

    print("[GRASP_TEST]: lifting")
    lift_target = dict(RAW_FULL_WAYPOINTS[-1])
    lift_target["shoulder_lift.pos"] += LIFT_SHOULDER_LIFT_DELTA_RAW
    ramp_to(lift_target, JAW_CLOSED_RAW, 2.5, "lift", RAW_FULL_WAYPOINTS[-1], JAW_CLOSED_RAW)
    for i in range(30):
        simulation_app.update()
        log("lift-hold", i)
        time.sleep(1.0 / 30.0)

    final_pos = cube_z_attr.Get()
    lifted = final_pos[2] > 0.7754 + 0.02  # more than 2cm above rest = genuine dynamic grasp
    print(f"[GRASP_TEST]: RESULT final cube_pos={list(final_pos)}, lifted={lifted}")

    timeline.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
