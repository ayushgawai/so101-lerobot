"""Bakes a keyboard_agent.py --action_log episode (.npz) into time-sampled
PhysX joint-drive targets on a reference to the robot USD, so the recorded
trajectory replays by hitting Play in USD Composer's own physics simulation.
No Isaac Lab involved on the Composer side.

Each of the 6 SO-101 joints in SO-ARM101-USD.usd is a PhysicsRevoluteJoint
with a `PhysicsDriveAPI:angular` applying `drive:angular:physics:targetPosition`
(confirmed by direct inspection: authored in DEGREES, matching lowerLimit/
upperLimit -- e.g. Rotation's [-110, 110] matches lerobot_interface.py's
SO101_USD_MAPPING shoulder_pan range exactly). The action log stores radians
(same convention env.step() uses), so this converts radians -> degrees.

Usage (needs `usd-core`, not this repo's Isaac Lab venv -- see
docs/isaac-sim-windows-guide.md section 7 for the throwaway-venv setup):

    python bake_action_log_to_usd_animation.py <episode.npz> <output.usd>

Then open <output.usd> in Composer and hit Play (physics simulation) to
watch the arm drive through the recorded trajectory.
"""
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

NPZ_PATH = sys.argv[1]
OUT_PATH = sys.argv[2]

ROBOT_REL_PATH = "../source/sim_to_real_so101/assets/usd/SO-ARM101-USD.usd"
JOINT_ORDER = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
FPS = 60.0  # matches keyboard_agent.py's per-step cadence (env step-size = 1/60 s)

log = np.load(NPZ_PATH)
actions = log["actions"]  # (T, 6) radians
logged_joint_names = list(log["joint_names"])
if logged_joint_names != JOINT_ORDER:
    raise ValueError(f"Unexpected joint order {logged_joint_names}, expected {JOINT_ORDER}")

num_frames = actions.shape[0]
print(f"Loaded {num_frames} frames from {NPZ_PATH}")

stage = Usd.Stage.CreateNew(OUT_PATH)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetStartTimeCode(0)
stage.SetEndTimeCode(max(num_frames - 1, 0))
stage.SetTimeCodesPerSecond(FPS)
stage.SetFramesPerSecond(FPS)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

robot_xform = UsdGeom.Xform.Define(stage, "/World/Robot")
robot_xform.GetPrim().GetReferences().AddReference(ROBOT_REL_PATH)

for joint_index, joint_name in enumerate(JOINT_ORDER):
    joint_path = f"/World/Robot/joints/{joint_name}"
    joint_prim = stage.OverridePrim(joint_path)

    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    if not drive_api:
        raise RuntimeError(
            f"No angular PhysicsDriveAPI found on {joint_path} -- unexpected robot USD structure"
        )
    target_pos_attr = drive_api.GetTargetPositionAttr()

    degrees_per_frame = np.degrees(actions[:, joint_index])
    for frame in range(num_frames):
        target_pos_attr.Set(float(degrees_per_frame[frame]), Usd.TimeCode(frame))

stage.GetRootLayer().Save()
print(f"Wrote {OUT_PATH}")

# Verify: reopen and spot-check a few time samples against the source log.
check_stage = Usd.Stage.Open(OUT_PATH)
for joint_index, joint_name in enumerate([JOINT_ORDER[0], JOINT_ORDER[-1]]):
    real_index = JOINT_ORDER.index(joint_name)
    prim = check_stage.GetPrimAtPath(f"/World/Robot/joints/{joint_name}")
    drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
    attr = drive_api.GetTargetPositionAttr()
    for frame in [0, num_frames // 2, num_frames - 1]:
        got = attr.Get(Usd.TimeCode(frame))
        expected = float(np.degrees(actions[frame, real_index]))
        status = "OK" if abs(got - expected) < 1e-3 else "MISMATCH"
        print(f"  [{status}] {joint_name} frame {frame}: got={got:.4f} expected={expected:.4f}")
