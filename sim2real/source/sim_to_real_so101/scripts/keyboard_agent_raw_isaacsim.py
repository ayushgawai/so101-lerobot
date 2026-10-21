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
"""Keyboard-jog teleop against raw Isaac Sim 6.0.1 -- NO Isaac Lab.

Every other script in this repo (keyboard_agent.py, lerobot_agent.py, etc.)
is built on isaaclab.app.AppLauncher / isaaclab.envs.ManagerBasedRLEnv. This
script exists because Isaac Lab is off-limits entirely in this context, and
the only Isaac Lab release that supports Isaac Sim 6.0.1 is Isaac Lab 3.0
(Beta) -- there is no Isaac-Lab-based way to satisfy "Isaac Sim 6.0.1, no
Isaac Lab". So this drives the robot directly against Isaac Sim's own APIs:
`isaacsim.SimulationApp` (not AppLauncher) and raw PhysX `PhysicsDriveAPI`
attributes on each joint (not ArticulationCfg/ImplicitActuatorCfg).

Run with a plain Isaac Sim Python that has no isaaclab installed, by direct
file path -- NOT `-m`, which fails here (this repo's package isn't
pip-installed in a plain Isaac Sim python, so `-m package.module` can't
resolve it before this file's own sys.path insertion below even runs):
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\keyboard_agent_raw_isaacsim.py

Scope (deliberately minimal): keyboard-jog joint control only. No gym env,
no contact-sensor grasp/placement detection, no dataset recording -- those
are all Isaac Lab manager/task-config concepts with no raw-Isaac-Sim
equivalent built here. See docs/aws-cube-to-bowl-teleop-plan.md for what the
Isaac-Lab-based version (Stage 1/2) additionally provides.
"""
import argparse
import os
import sys

# Make sim_to_real_so101 importable without requiring an editable pip install
# in whatever Python this script is run with (this repo's package is only
# pip-installed into the Isaac-Lab venvs, not into a plain Isaac Sim python).
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)

from isaacsim import SimulationApp  # noqa: E402

parser = argparse.ArgumentParser(description="Raw Isaac Sim SO-101 keyboard-jog teleop agent (no Isaac Lab).")
parser.add_argument("--headless", action="store_true", default=False, help="Run without a viewport window.")
parser.add_argument(
    "--joint_step",
    type=float,
    default=0.5,
    help="Degrees added/removed from a joint's target per physics step while its jog key is held.",
)
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

"""Rest everything follows -- Kit is booted, pxr/omni/carb are importable now."""

import carb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

from sim_to_real_so101.utils.keyboard import JointJogKeyboardControl  # noqa: E402
from sim_to_real_so101.utils.physics_material import apply_friction_material  # noqa: E402
from sim_to_real_so101.utils.scene_reset import restore_prim_pose, snapshot_xform_ops  # noqa: E402
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.join(_DEMO_DIR, "real-to-sim.usd")

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
AWS_CUBE_PRIM_PATH = "/World/AWSBuilderCube"
AWS_CUBE_MASS_KG = 0.05
# Collision mesh, not the AWS_CUBE_PRIM_PATH Xform -- friction/restitution are
# shape properties (PhysicsMaterialAPI belongs on the prim with
# PhysicsCollisionAPI), not body properties (RigidBodyAPI/MassAPI, which
# belong on the rigid-body root above).
AWS_CUBE_COLLISION_MESH_PATH = "/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo"
PAPER_BOWL_PRIM_PATH = "/World/PaperBowl"

# Gripper pad collision shapes -- Xform prims (not Mesh) with
# PhysicsCollisionAPI/PhysicsMeshCollisionAPI applied directly, confirmed via
# direct inspection of real-to-sim.usd (inspect_gripper_collision.py).
# "gripper" is the fixed pad, "jaw" the pincer that actually moves.
GRIPPER_COLLISION_PATH = "/World/SO_ARM101_USD/gripper/collisions"
JAW_COLLISION_PATH = "/World/SO_ARM101_USD/jaw/collisions"

# Rigid PVC-like friction for the cube (typical dry μs ~0.4–0.5, μk ~0.35–0.40).
# Matches PhysicsMaterialAPI authored on real-to-sim.usd's AWSBuilderCube_Geo.
AWS_CUBE_STATIC_FRICTION = 0.45
AWS_CUBE_DYNAMIC_FRICTION = 0.40
AWS_CUBE_RESTITUTION = 0.0

# FDM 3D-printed PLA/PETG friction for the jaw/gripper pads (physical SO-ARM101
# parts) -- typical dry μs ~0.35–0.40, μk ~0.25–0.35. Matches PhysicsMaterialAPI
# authored on real-to-sim.usd's gripper/jaw collision prims.
GRIPPER_STATIC_FRICTION = 0.40
GRIPPER_DYNAMIC_FRICTION = 0.35
GRIPPER_RESTITUTION = 0.0

# Exact xformOp:translate / xformOp:orient authored on /World/SO_ARM101_USD
# in real-to-sim.usd (verified directly against the raw prim -- same values
# used by the Isaac-Lab-based task's ROBOT_POS/ROBOT_ROT).
ROBOT_POS = Gf.Vec3f(0.0, 0.3, 0.72)

# Isaac-Lab-tuned actuator gains, ported from SO101_CFG in assets/so101.py
# (isaaclab.actuators.ImplicitActuatorCfg). Used AS-IS, unconverted, on the
# raw PhysicsDriveAPI attributes -- an earlier version of this script scaled
# these by pi/180 on the theory that the drive gain operates against the
# joint's degree-valued position representation (matching its authored
# lowerLimit/upperLimit). That was empirically wrong: it made every joint
# far too weak to generate real grip torque against an obstruction (e.g.
# Jaw's converted stiffness of ~0.07 produces ~1.4 N*m against a plausible
# 20deg blocked error -- nowhere near its 30 N*m maxForce -- while the
# unconverted value of 4 produces ~80 N*m, correctly clamped to the limit).
# Confirmed via a live A/B/C step-response test (check_drive_gain_units.py):
# unconverted Isaac-Lab values tracked a 60deg step target just as well as
# the raw file's own default gains, while the pi/180-converted values were
# consistently the least accurate of the three. PhysX's drive equation
# evidently operates on radians internally regardless of how the joint's
# position/limits are authored/displayed in the USD file.
JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    # Jaw's effort_limit was 30 (copy-pasted from the other 5 joints,
    # untuned for load) until a user report: fully closing the jaw on
    # AWSBuilderCube launched it away instead of holding it. Root cause,
    # confirmed via diag_raw_grasp_instability.py (C:\ilab): commanding full
    # closure against the cube (a target the PD drive can't physically
    # reach) builds squeeze force up to the effort_limit every tick, and
    # 30 N*m is wildly more than this 0.05kg cube needs -- holding its
    # weight via friction takes roughly 0.01 N*m, so 30 N*m destabilizes
    # the contact solve and ejects it. Measured 1.17m of cube drift/launch
    # at effort_limit=30 vs 0.24m (no violent launch) at effort_limit=3 in
    # an identical A/B trial. 3 N*m is still ~250x the cube's actual holding
    # requirement, so grip strength isn't traded away by this reduction.
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=3),
}


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(REAL_TO_SIM_USD)
    simulation_app.update()
    stage = usd_context.get_stage()

    # The raw file authors no RigidBodyAPI/MassAPI on the cube (plan §2) --
    # without this it would behave like a static wall under gripper contact.
    cube_prim = stage.GetPrimAtPath(AWS_CUBE_PRIM_PATH)
    if not cube_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_PRIM_PATH}")
    if not cube_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    if not cube_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(cube_prim).CreateMassAttr(AWS_CUBE_MASS_KG)

    cube_collision_mesh = stage.GetPrimAtPath(AWS_CUBE_COLLISION_MESH_PATH)
    if not cube_collision_mesh.IsValid():
        raise RuntimeError(f"Expected prim not found: {AWS_CUBE_COLLISION_MESH_PATH}")
    apply_friction_material(
        cube_collision_mesh, AWS_CUBE_STATIC_FRICTION, AWS_CUBE_DYNAMIC_FRICTION, AWS_CUBE_RESTITUTION
    )

    for collision_path in (GRIPPER_COLLISION_PATH, JAW_COLLISION_PATH):
        collision_prim = stage.GetPrimAtPath(collision_path)
        if not collision_prim.IsValid():
            raise RuntimeError(f"Expected prim not found: {collision_path}")
        apply_friction_material(
            collision_prim, GRIPPER_STATIC_FRICTION, GRIPPER_DYNAMIC_FRICTION, GRIPPER_RESTITUTION
        )

    bowl_prim = stage.GetPrimAtPath(PAPER_BOWL_PRIM_PATH)
    if not bowl_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {PAPER_BOWL_PRIM_PATH}")

    # Captured before the timeline plays, so this is each prim's originally
    # authored pose -- restored on 'R' alongside the joint-target reset below.
    cube_orig_pose = snapshot_xform_ops(cube_prim)
    bowl_orig_pose = snapshot_xform_ops(bowl_prim)

    # root_joint (a PhysicsFixedJoint, empty body0 = binds to the physics
    # scene's literal origin, NOT the ancestor Xform) has a non-identity
    # localRot1 baked in on the robot side, but localPos0/localRot0 (the
    # world side) are left at identity -- i.e. this joint was authored
    # assuming /World/SO_ARM101_USD sits AT the origin. It doesn't here
    # (translated to ROBOT_POS), so without this fix PhysX logs "found a
    # joint with disjointed body transforms" and snaps the whole robot
    # toward wherever the joint's constraint actually resolves -- nowhere
    # near the table. Compensating localPos0/localRot0 so the constraint is
    # already satisfied at the robot's real, authored position: since
    # localPos1 is (0,0,0) and the ancestor Xform's own orientation is
    # identity, the fix is just localPos0=ROBOT_POS,
    # localRot0=(the joint's own existing localRot1, reused rather than
    # reinvented) -- confirmed by direct inspection of real-to-sim.usd.
    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    if not root_joint_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    root_joint_prim.GetAttribute("physics:localPos0").Set(ROBOT_POS)
    root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

    # Apply the tuned actuator gains and cache each joint's limits/attributes.
    joints = {}
    for joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Expected joint prim not found: {joint_name}")

        joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
        joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
        joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))

        target_attr = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        lower = joint_prim.GetAttribute("physics:lowerLimit").Get()
        upper = joint_prim.GetAttribute("physics:upperLimit").Get()
        joints[joint_name] = {
            "target_attr": target_attr,
            "target": target_attr.Get() or 0.0,
            "lower": lower,
            "upper": upper,
        }

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    keyboard_control = JointJogKeyboardControl()

    print_simulator_version_banner()
    print("[INFO]: Keyboard jog controls (hold to move, release to stop):")
    for joint in JointJogKeyboardControl.JOINT_ORDER:
        pos_key, neg_key = JointJogKeyboardControl.JOINT_KEYS[joint]
        print(f"    {joint:<12s}  {pos_key} (+)  /  {neg_key} (-)")
    print("[INFO]: Click 'R' to reset all joint targets to 0 degrees, and the")
    print("        cube/bowl to their original positions")

    try:
        while simulation_app.is_running():
            deltas = keyboard_control.get_joint_deltas(args_cli.joint_step)

            if keyboard_control.reset_world:
                for info in joints.values():
                    info["target"] = 0.0
                restore_prim_pose(cube_prim, cube_orig_pose, zero_velocity=True)
                restore_prim_pose(bowl_prim, bowl_orig_pose)
                keyboard_control.reset_world = False

            for joint_name, delta in zip(JointJogKeyboardControl.JOINT_ORDER, deltas):
                info = joints[joint_name]
                new_target = info["target"] + delta
                info["target"] = max(info["lower"], min(info["upper"], new_target))

            for info in joints.values():
                info["target_attr"].Set(info["target"])

            simulation_app.update()
    finally:
        keyboard_control.cleanup()
        timeline.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
