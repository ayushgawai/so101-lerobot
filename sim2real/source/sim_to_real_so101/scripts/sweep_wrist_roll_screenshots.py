"""Load the sim stage, sweep the Wrist_Roll joint through several target
angles, and screenshot the viewport at each -- NO real hardware connected at
all, so none of the serial/lerobot code runs and there is zero risk of the
follower-connect crash this repo has been chasing elsewhere.

Purpose: determine what wrist_roll degree value the SIM considers "jaws open
sideways" vs "jaws open up-down", to compare against the real arm's reported
degree at the same physical pose (see stream_leader_follower_wrist_roll.py).

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\sweep_wrist_roll_screenshots.py
Screenshots land in the scratch dir printed at the end.
"""
import os

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_REPO_ROOT_DIR = os.path.dirname(_REPO_SOURCE_DIR)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": False})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from omni.kit.viewport.utility import (  # noqa: E402
    get_active_viewport,
    get_active_viewport_camera_path,
    capture_viewport_to_file,
)

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
REAL_TO_SIM_USD = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo", "real-to-sim.usd")
OUT_DIR = os.path.join(os.environ.get("TEMP", _REPO_ROOT_DIR), "wrist_roll_sweep")
os.makedirs(OUT_DIR, exist_ok=True)

usd_context = omni.usd.get_context()
usd_context.open_stage(REAL_TO_SIM_USD)
simulation_app.update()
stage = usd_context.get_stage()

# Same root_joint mount patch every other script in this repo applies --
# without it the articulation is unconstrained (see their comments).
root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
if not root_joint_prim.IsValid():
    raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/root_joint")
local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
root_joint_prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.0, 0.3, 0.72))
root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)

wrist_roll_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/Wrist_Roll")
if not wrist_roll_prim.IsValid():
    raise RuntimeError(f"Expected prim not found: {ROBOT_PRIM_PATH}/joints/Wrist_Roll")
wrist_roll_prim.GetAttribute("drive:angular:physics:stiffness").Set(7.0)
wrist_roll_prim.GetAttribute("drive:angular:physics:damping").Set(0.5)
wrist_roll_prim.GetAttribute("drive:angular:physics:maxForce").Set(30.0)
target_attr = wrist_roll_prim.GetAttribute("drive:angular:physics:targetPosition")
lower = wrist_roll_prim.GetAttribute("physics:lowerLimit").Get()
upper = wrist_roll_prim.GetAttribute("physics:upperLimit").Get()
print(f"Wrist_Roll limits: [{lower}, {upper}] deg", flush=True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(30):
    simulation_app.update()

viewport = get_active_viewport()

# The default saved camera (whatever real-to-sim.usd shipped with) framed
# nothing useful -- an extreme close-up looking almost straight up at the
# ceiling, gripper tips barely in frame, no visible change across the whole
# sweep. Point the viewport camera explicitly at the gripper instead: world
# position ~(0.02, 0.02, 0.99) per direct USD inspection (Xformable
# ComputeLocalToWorldTransform on /World/SO_ARM101_USD/gripper), pulled back
# along -Y and raised slightly for a clear side view of the jaw.
#
# Built with Gf.Matrix4d.SetLookAt (a standard, verified USD utility) rather
# than hand-derived Euler angles -- confirmed separately that
# SetLookAt(eye, center, up).GetInverse() correctly recovers `eye` as the
# resulting camera-to-world translation before relying on it here.
camera_path = get_active_viewport_camera_path(viewport.usd_context_name)
camera_prim = stage.GetPrimAtPath(camera_path)
if not camera_prim.IsValid():
    raise RuntimeError(f"Active viewport camera prim not found at {camera_path}")
GRIPPER_WORLD_POS = Gf.Vec3d(0.02, 0.02, 0.99)
EYE_POS = GRIPPER_WORLD_POS + Gf.Vec3d(0.0, -0.5, 0.1)
view_matrix = Gf.Matrix4d().SetLookAt(EYE_POS, GRIPPER_WORLD_POS, Gf.Vec3d(0, 0, 1))
camera_to_world = view_matrix.GetInverse()

camera_xformable = UsdGeom.Xformable(camera_prim)
camera_xformable.ClearXformOpOrder()
camera_xformable.AddTransformOp().Set(camera_to_world)
for _ in range(10):
    simulation_app.update()

for deg in (0, 45, 90, -45, -90, 135, -135):
    clamped = max(lower, min(upper, float(deg)))
    target_attr.Set(clamped)
    for _ in range(30):
        simulation_app.update()
    out_path = os.path.join(OUT_DIR, f"wrist_roll_{deg:+04d}deg.png")
    capture_viewport_to_file(viewport, out_path)
    # capture_viewport_to_file returns a future; no clean sync wait available
    # here, so give it generous settle ticks instead. If an image comes out
    # blank/corrupt, that's the tell -- rerun with more ticks for that value.
    for _ in range(60):
        simulation_app.update()
    print(f"captured deg={deg:+d} (clamped={clamped:.1f}) -> {out_path}", flush=True)

print(f"\nDONE. Screenshots in: {OUT_DIR}", flush=True)
timeline.stop()
simulation_app.close()
