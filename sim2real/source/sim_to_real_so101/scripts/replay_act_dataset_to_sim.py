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
"""Replays a recorded lerobot-record dataset (ACT eval rollout, or any
so_follower recording) into raw Isaac Sim, joint-for-joint -- Step 2a of the
real->sim mirroring plan (see docs -- ArUco tracking dropped in favor of
measured object positions + this replay).

This is a validation/interim milestone, not a live demo: it plays back a
*past* rollout, so the sim isn't reacting to anything live. It proves the
joint->sim mapping and grasp physics are correct before attempting a live
stream (Phase 2b). If a live stream never gets built, this is also the
honest fallback demo -- "here's a recorded rollout replayed in sim," not
"watch it live."

Drives sim joints from each frame's `observation.state` (measured, not
`action`/commanded) -- state lags action by a few units per joint in real
recordings (see the plan), and the sim should show where the arm *was*, not
where it was told to go.

Structurally this is leader_arm_teleop_raw_isaacsim.py with the leader-arm
serial read swapped for a dataframe row -- same stage setup, joint gains,
friction materials, and measured cube/bowl positions, lifted directly rather
than reimplemented.

Reads the dataset's parquet files directly with pandas/pyarrow, not via
`lerobot` -- no HF cache dance, no extra dependency in Isaac Sim's python.
Same approach so101-lerobot/scripts/measure_placement_error.py already
takes.

Run with the same Isaac Sim Python as the teleop script, by direct file
path:
    C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\replay_act_dataset_to_sim.py --dataset <path>

--dataset points at a lerobot v3 dataset root (containing meta/ and data/),
e.g. a directory under ~/.cache/huggingface/lerobot/<user>/<repo>. --episode
picks which recorded episode to replay (default 0). --loop repeats it
(cube/bowl reset between loops, same as the teleop script's 'R' key) for a
standing demo table.
"""
import argparse
import glob
import os
import sys
import time

# Make sim_to_real_so101 importable without requiring an editable pip install
# (matches leader_arm_teleop_raw_isaacsim.py).
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_SOURCE_DIR not in sys.path:
    sys.path.insert(0, _REPO_SOURCE_DIR)

parser = argparse.ArgumentParser(description="Replay a recorded lerobot dataset's joint trajectory into raw Isaac Sim.")
parser.add_argument("--headless", action="store_true", default=False, help="Run without a viewport window.")
parser.add_argument(
    "--dataset",
    type=str,
    required=True,
    help="Path to a lerobot v3 dataset root (contains meta/ and data/), e.g. a directory under "
    "~/.cache/huggingface/lerobot/<user>/<repo>.",
)
parser.add_argument("--episode", type=int, default=0, help="Episode index within the dataset to replay.")
parser.add_argument("--loop", action="store_true", default=False, help="Repeat the episode until closed.")
parser.add_argument(
    "--cube_pos",
    type=str,
    default=None,
    help="Override the cube's world position as 'x,y,z' in meters. Unset (default) = use the measured "
    "constant (same as leader_arm_teleop_raw_isaacsim.py's REAL_CUBE_POS) -- note that constant reflects "
    "the *current* physical setup, not necessarily wherever this recorded episode's real cube was, so the "
    "replayed grasp may not land exactly on it. Override this if you know where it was.",
)
parser.add_argument(
    "--bowl_pos",
    type=str,
    default=None,
    help="Override the bowl's world position as 'x,y,z' in meters. Unset (default) = use the measured "
    "constant. Same caveat as --cube_pos.",
)
parser.add_argument(
    "--track_cube_height",
    action="store_true",
    default=False,
    help="Debug: print the cube's world Z every 10 frames, to numerically confirm a grasp/carry/release "
    "happened (Z rises off the resting height, then settles at a different height) without needing to "
    "eyeball the viewport at the right moment. Off by default -- no per-tick spam in normal demo runs.",
)
parser.add_argument(
    "--release_test_frame",
    type=int,
    default=None,
    help="Debug: once this frame is reached while the cube is latched, force-release it back to "
    "dynamic (kinematicEnabled=false) exactly once and never re-latch for the rest of the run -- "
    "tests whether contact/friction physics alone sustains an already-seated grasp once the "
    "kinematic latch is no longer overriding its pose. Requires the latch to actually be holding "
    "at that frame (see the [GRASP] HOLD-started log lines to pick a frame inside a hold window).",
)
parser.add_argument(
    "--no_grasp_latch",
    action="store_true",
    default=False,
    help="Disable the Step 3 grasp-latch fallback (on by default). Confirmed necessary empirically: the "
    "sim cube sits at the *current* measured real-world position, not necessarily wherever this "
    "recorded episode's real cube was, so relying on sim contact dynamics alone usually whiffs the grasp. "
    "With the latch on, a sustained real gripper hold (detected from commanded-vs-measured gripper gap, "
    "see utils/grasp_events.py) makes the cube follow the gripper rigidly regardless of sim contact. Pass "
    "this flag to see the raw (usually unconvincing) sim-physics-only behavior instead.",
)
args_cli = parser.parse_args()


def _validate_vec3_str(arg_name: str, value: str | None) -> None:
    """Fail fast on a malformed --cube_pos/--bowl_pos before paying for Kit's
    ~30s+ boot below, matching leader_arm_teleop_raw_isaacsim.py's identical check."""
    if value is None:
        return
    parts = value.split(",")
    if len(parts) != 3:
        print(
            f"[ERROR]: --{arg_name} must be 'x,y,z' (three comma-separated numbers), got: {value!r}",
            file=sys.stderr,
        )
        sys.exit(1)
    try:
        [float(p) for p in parts]
    except ValueError:
        print(f"[ERROR]: --{arg_name} must be numeric 'x,y,z', got: {value!r}", file=sys.stderr)
        sys.exit(1)


_validate_vec3_str("cube_pos", args_cli.cube_pos)
_validate_vec3_str("bowl_pos", args_cli.bowl_pos)

if not os.path.isdir(args_cli.dataset):
    print(f"[ERROR]: --dataset directory not found: {args_cli.dataset}", file=sys.stderr)
    sys.exit(1)

# Fail fast on a bad --episode / dataset layout before paying for Kit's
# ~30s+ boot below -- same reasoning as the teleop script's calibration-file
# checks. pandas/pyarrow only, no lerobot/torch/pxr needed yet.
import pandas as pd  # noqa: E402

_EPISODE_META_FILES = glob.glob(os.path.join(args_cli.dataset, "meta", "episodes", "**", "*.parquet"), recursive=True)
if not _EPISODE_META_FILES:
    print(f"[ERROR]: No episode metadata under {args_cli.dataset}/meta/episodes -- is this a lerobot v3 dataset?", file=sys.stderr)
    sys.exit(1)

_episodes_meta = pd.concat([pd.read_parquet(f) for f in _EPISODE_META_FILES], ignore_index=True)
_episode_rows = _episodes_meta[_episodes_meta["episode_index"] == args_cli.episode]
if _episode_rows.empty:
    _available = sorted(_episodes_meta["episode_index"].unique().tolist())
    print(
        f"[ERROR]: --episode {args_cli.episode} not found in {args_cli.dataset}. "
        f"Available episodes: {_available}",
        file=sys.stderr,
    )
    sys.exit(1)
_episode_meta = _episode_rows.iloc[0]

_DATA_CHUNK = int(_episode_meta["data/chunk_index"])
_DATA_FILE = int(_episode_meta["data/file_index"])
_DATA_PARQUET = os.path.join(args_cli.dataset, "data", f"chunk-{_DATA_CHUNK:03d}", f"file-{_DATA_FILE:03d}.parquet")
if not os.path.isfile(_DATA_PARQUET):
    print(f"[ERROR]: Data file not found: {_DATA_PARQUET}", file=sys.stderr)
    sys.exit(1)

_episode_frames = pd.read_parquet(_DATA_PARQUET)
_episode_frames = _episode_frames[_episode_frames["episode_index"] == args_cli.episode].sort_values("frame_index")
if _episode_frames.empty:
    print(f"[ERROR]: Episode {args_cli.episode} has no frames in {_DATA_PARQUET}.", file=sys.stderr)
    sys.exit(1)

_STATE_SEQUENCE = _episode_frames["observation.state"].tolist()  # list of length-6 float arrays
_ACTION_SEQUENCE = _episode_frames["action"].tolist()  # commanded 6-D, used only by the grasp-latch below

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args_cli.headless})

"""Rest everything follows -- Kit is booted, pxr/omni/carb are importable now."""

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import torch  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

from sim_to_real_so101.utils.grasp_events import GraspDetector  # noqa: E402
from sim_to_real_so101.utils.keyboard import KeyboardControl  # noqa: E402
from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface  # noqa: E402
from sim_to_real_so101.utils.physics_material import apply_friction_material  # noqa: E402
from sim_to_real_so101.utils.scene_reset import restore_prim_pose, snapshot_xform_ops  # noqa: E402
from sim_to_real_so101.utils.version_banner import print_simulator_version_banner  # noqa: E402
from sim_to_real_so101.utils.xform_prim_compat import make_xform_prim  # noqa: E402

print_simulator_version_banner()

_DEMO_DIR = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo")
REAL_TO_SIM_USD = os.path.join(_DEMO_DIR, "real-to-sim.usd")

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
AWS_CUBE_PRIM_PATH = "/World/AWSBuilderCube"
AWS_CUBE_MASS_KG = 0.05
AWS_CUBE_COLLISION_MESH_PATH = "/World/AWSBuilderCube/Geometry/AWSBuilderCube_Geo"
PAPER_BOWL_PRIM_PATH = "/World/PaperBowl"

GRIPPER_COLLISION_PATH = "/World/SO_ARM101_USD/gripper/collisions"
JAW_COLLISION_PATH = "/World/SO_ARM101_USD/jaw/collisions"

# Same friction tuning as leader_arm_teleop_raw_isaacsim.py -- PVC cube +
# 3D-printed jaw/gripper (matches PhysicsMaterialAPI on real-to-sim.usd).
AWS_CUBE_STATIC_FRICTION = 0.45
AWS_CUBE_DYNAMIC_FRICTION = 0.40
AWS_CUBE_RESTITUTION = 0.0
GRIPPER_STATIC_FRICTION = 0.40
GRIPPER_DYNAMIC_FRICTION = 0.35
GRIPPER_RESTITUTION = 0.0

ROBOT_POS = Gf.Vec3f(0.0, 0.3, 0.72)

# Measured 2026-08-19 -- identical to leader_arm_teleop_raw_isaacsim.py's
# REAL_CUBE_POS/REAL_BOWL_POS. Kept as a separate copy here (not imported)
# since that script has its own argparse/SimulationApp-launch side effects
# at import time that would conflict with this script's own args_cli.
REAL_CUBE_POS = Gf.Vec3d(0.0, 0.047, 0.7754)
REAL_BOWL_POS = Gf.Vec3d(0.153, 0.047, 0.7500)

# Same tuned gains as leader_arm_teleop_raw_isaacsim.py -- see that file's
# JOINT_GAINS for the full empirical justification. Order doubles as the
# SO101_JOINT_ORDER-positional correspondence for mapped actions.
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

STATS_PRINT_INTERVAL_S = 5.0


def _parse_vec3(value: str) -> Gf.Vec3d:
    x, y, z = (float(p) for p in value.split(","))
    return Gf.Vec3d(x, y, z)


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

    cube_pos = _parse_vec3(args_cli.cube_pos) if args_cli.cube_pos else REAL_CUBE_POS
    bowl_pos = _parse_vec3(args_cli.bowl_pos) if args_cli.bowl_pos else REAL_BOWL_POS
    cube_prim.GetAttribute("xformOp:translate").Set(cube_pos)
    bowl_prim.GetAttribute("xformOp:translate").Set(bowl_pos)

    cube_orig_pose = snapshot_xform_ops(cube_prim)
    bowl_orig_pose = snapshot_xform_ops(bowl_prim)

    root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
    if not root_joint_prim.IsValid():
        raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
    local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
    root_joint_prim.GetAttribute("physics:localPos0").Set(ROBOT_POS)
    root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

    joints = {}
    for joint_name, gains in JOINT_GAINS.items():
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Expected joint prim not found: {joint_name}")

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
    timeline.play()
    simulation_app.update()

    # Bare interface, never connected -- only used for its stateless
    # get_raw_actions_tensor/get_mapped_actions_vectorized conversion
    # (same SO101_USD_MAPPING the teleop script's live loop uses).
    conv_iface = LeRobotSO101Interface(device="cpu", port=None, id="unused", cameras={}, fps=30, kind="leader")

    keyboard_control = KeyboardControl()

    print_simulator_version_banner()
    print(f"[INFO]: Replaying episode {args_cli.episode} from {args_cli.dataset} ({len(_STATE_SEQUENCE)} frames).")
    print("[INFO]: Click 'R' to reset the cube/bowl to their measured positions. Ctrl+C or close the window to stop.")

    frame_period_s = 1.0 / 30.0  # dataset fps (info.json) -- real-time pacing for a watchable demo
    tick_count = 0
    tick_time_total = 0.0
    last_stats_print = time.perf_counter()
    frame_idx = 0
    cube_translate_attr = cube_prim.GetAttribute("xformOp:translate")
    _diag_gripper_view = make_xform_prim(f"{ROBOT_PRIM_PATH}/gripper") if args_cli.track_cube_height else None
    _diag_jaw_view = make_xform_prim(f"{ROBOT_PRIM_PATH}/jaw") if args_cli.track_cube_height else None

    # Step 3 grasp-latch fallback -- see utils/grasp_events.py and the
    # --no_grasp_latch help text above for why this is on by default.
    # gripper_view.get_world_poses() returns (positions (N,3), orientations
    # (N,4) wxyz) as numpy arrays -- confirmed directly against this stage,
    # not assumed. Position-only latch (cube keeps its orientation frozen at
    # whatever it was when the hold started) -- simpler and lower-risk than
    # also tracking rotation, and the cube has no orient op to write to
    # anyway (see Step 1's optional-orientation note).
    grasp_latch_enabled = not args_cli.no_grasp_latch
    gripper_view = make_xform_prim(f"{ROBOT_PRIM_PATH}/gripper") if grasp_latch_enabled else None
    grasp_detector = GraspDetector()
    was_holding = False
    latch_offset = None  # Gf.Vec3d, cube_pos - gripper_pos, captured at hold-onset
    release_test_fired = False

    try:
        playing = True
        while simulation_app.is_running() and playing:
            for state, action in zip(_STATE_SEQUENCE, _ACTION_SEQUENCE):
                tick_start = time.perf_counter()

                if args_cli.track_cube_height and frame_idx % 10 == 0:
                    cz = cube_translate_attr.Get()[2]
                    gpos, _ = _diag_gripper_view.get_world_poses()
                    gx, gy, gz = gpos[0].tolist()
                    jpos, _ = _diag_jaw_view.get_world_poses()
                    jx, jy, jz = jpos[0].tolist()
                    print(
                        f"[TRACK]: frame={frame_idx} cube_z={cz:.4f} (rest={REAL_CUBE_POS[2]:.4f}) "
                        f"gripper=({gx:.4f},{gy:.4f},{gz:.4f}) jaw=({jx:.4f},{jy:.4f},{jz:.4f})"
                    )
                frame_idx += 1

                if keyboard_control.reset_world:
                    restore_prim_pose(cube_prim, cube_orig_pose, zero_velocity=True)
                    restore_prim_pose(bowl_prim, bowl_orig_pose)
                    keyboard_control.reset_world = False
                    was_holding = False
                    latch_offset = None
                    if grasp_latch_enabled:
                        UsdPhysics.RigidBodyAPI(cube_prim).CreateKinematicEnabledAttr().Set(False)

                real_state = {name: float(value) for name, value in zip(conv_iface.SO101_JOINT_ORDER, state)}
                raw_tensor = conv_iface.get_raw_actions_tensor(real_state)
                mapped_deg = conv_iface.get_mapped_actions_vectorized(raw_tensor) * RAD_TO_DEG

                for joint_name, target_deg in zip(JOINT_ORDER, mapped_deg.tolist()):
                    info = joints[joint_name]
                    clamped = max(info["lower"], min(info["upper"], target_deg))
                    info["target_attr"].Set(clamped)

                simulation_app.update()

                if grasp_latch_enabled:
                    commanded_grip = float(action[5])
                    measured_grip = float(state[5])
                    now_holding = grasp_detector.update(commanded_grip, measured_grip)

                    if now_holding and not was_holding and not release_test_fired:
                        UsdPhysics.RigidBodyAPI(cube_prim).CreateKinematicEnabledAttr().Set(True)
                        grip_pos_np, _ = gripper_view.get_world_poses()
                        grip_pos = Gf.Vec3d(*grip_pos_np[0].tolist())
                        latch_offset = Gf.Vec3d(cube_translate_attr.Get()) - grip_pos
                        print(f"[GRASP]: frame={frame_idx - 1} HOLD started (cube latched to gripper)")
                    elif was_holding and not now_holding:
                        UsdPhysics.RigidBodyAPI(cube_prim).CreateKinematicEnabledAttr().Set(False)
                        latch_offset = None
                        print(f"[GRASP]: frame={frame_idx - 1} HOLD ended (cube released, back to dynamic)")

                    if now_holding and latch_offset is not None:
                        grip_pos_np, _ = gripper_view.get_world_poses()
                        grip_pos = Gf.Vec3d(*grip_pos_np[0].tolist())
                        cube_translate_attr.Set(grip_pos + latch_offset)

                    if (
                        args_cli.release_test_frame is not None
                        and not release_test_fired
                        and now_holding
                        and frame_idx - 1 >= args_cli.release_test_frame
                    ):
                        UsdPhysics.RigidBodyAPI(cube_prim).CreateKinematicEnabledAttr().Set(False)
                        release_test_fired = True
                        now_holding = False  # stop re-latching for the rest of the run
                        latch_offset = None
                        print(
                            f"[RELEASE_TEST]: frame={frame_idx - 1} forced release -- cube now purely "
                            "dynamic, watching whether contact/friction sustains the grip on its own"
                        )

                    was_holding = now_holding

                tick_count += 1
                tick_time_total += time.perf_counter() - tick_start
                now = time.perf_counter()
                if now - last_stats_print >= STATS_PRINT_INTERVAL_S:
                    avg_tick_ms = (tick_time_total / tick_count) * 1000.0
                    print(f"[INFO]: avg tick={avg_tick_ms:.2f} ms over {tick_count} ticks")
                    tick_count = 0
                    tick_time_total = 0.0
                    last_stats_print = now

                if not simulation_app.is_running():
                    playing = False
                    break

                elapsed = time.perf_counter() - tick_start
                if elapsed < frame_period_s:
                    time.sleep(frame_period_s - elapsed)

            if not args_cli.loop:
                playing = False
            elif playing:
                print("[INFO]: Episode complete, looping -- resetting cube/bowl.")
                restore_prim_pose(cube_prim, cube_orig_pose, zero_velocity=True)
                restore_prim_pose(bowl_prim, bowl_orig_pose)
    finally:
        keyboard_control.cleanup()
        timeline.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
