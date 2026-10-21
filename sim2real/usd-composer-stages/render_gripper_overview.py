"""Wide external view of the gripper area with a bright marker sphere at the
WristCamera's eye position and a thin rod along its boresight, so the
camera's placement relative to the physical camera_mount / jaw / gripper
body can be checked visually instead of guessed from macro close-ups shot
through the wrist camera itself.

Run with Isaac Sim's own Python:
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\render_gripper_overview.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import os
import shutil
import tempfile

from pxr import Usd, UsdGeom, Gf, Sdf
import omni.replicator.core as rep
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)
OUT_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop"
    r"\usd-composer-stages\wrist_camera_placement_overview.png"
)
OUT_DIR = tempfile.mkdtemp(prefix="gripper_overview_")
GRIPPER_PATH = "/World/SO_ARM101_USD/gripper"
WRIST_CAMERA_PATH = f"{GRIPPER_PATH}/WristCamera"

print(f"Opening {STAGE_PATH}")
open_stage(usd_path=STAGE_PATH)
while is_stage_loading():
    simulation_app.update()
print("Stage loaded")

import omni.usd
stage = omni.usd.get_context().get_stage()

xf_cache = UsdGeom.XformCache()
cam_prim = stage.GetPrimAtPath(WRIST_CAMERA_PATH)
cam_world = xf_cache.GetLocalToWorldTransform(cam_prim)
cam_pos = cam_world.ExtractTranslation()
cam_rot = cam_world.ExtractRotationMatrix()
cam_forward = -cam_rot.GetRow(2)
print("WristCamera world pos:", cam_pos)
print("WristCamera world forward:", cam_forward)

# Bright red marker sphere at the eye, small rods marking 1cm and 3cm ahead.
marker = UsdGeom.Sphere.Define(stage, "/World/DebugEyeMarker")
marker.CreateRadiusAttr(0.004)
UsdGeom.Xformable(marker).AddTranslateOp().Set(Gf.Vec3d(cam_pos))
marker.CreateDisplayColorAttr([Gf.Vec3f(1, 0, 0)])

for i, d in enumerate([0.01, 0.03, 0.06]):
    dot = UsdGeom.Sphere.Define(stage, f"/World/DebugRay{i}")
    dot.CreateRadiusAttr(0.003)
    p = cam_pos + Gf.Vec3d(cam_forward) * d
    UsdGeom.Xformable(dot).AddTranslateOp().Set(p)
    dot.CreateDisplayColorAttr([Gf.Vec3f(0, 1, 0)])

gripper_world = xf_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(GRIPPER_PATH))
gripper_pos = gripper_world.ExtractTranslation()
print("gripper world pos:", gripper_pos)
jaw_world = xf_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/SO_ARM101_USD/jaw"))
jaw_pos = jaw_world.ExtractTranslation()
print("jaw world pos:", jaw_pos)

# External orbit camera framing the whole arm from ~65cm away -- far enough
# that small errors in exactly where to aim don't cause total misses.
eye = Gf.Vec3d(gripper_pos[0] + 0.35, gripper_pos[1] - 0.45, gripper_pos[2] + 0.35)
target = Gf.Vec3d(gripper_pos[0], gripper_pos[1], gripper_pos[2] - 0.15)
up = Gf.Vec3d(0.0, 0.0, 1.0)
view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
cam_to_world = view_matrix.GetInverse()

overview_cam = UsdGeom.Camera.Define(stage, "/World/OverviewCheckCamera")
overview_cam.CreateFocalLengthAttr(15.0)
overview_cam.CreateClippingRangeAttr(Gf.Vec2f(0.001, 10.0))
xf = UsdGeom.Xformable(overview_cam)
xf.AddTransformOp().Set(Gf.Matrix4d(cam_to_world))

# Deliberately NOT calling timeline.play(): this stage's root_joint has a
# disjointed-body-transform PhysX warning on load, and playing physics snaps
# the robot away from its authored rest pose during the frames we'd be
# accumulating -- confirmed by an earlier render showing the debug markers
# floating with no robot in sight. We only want the authored/static geometry,
# so render with physics stopped.

render_product = rep.create.render_product("/World/OverviewCheckCamera", resolution=(1024, 1024))
writer = rep.WriterRegistry.get("BasicWriter")
os.makedirs(OUT_DIR, exist_ok=True)
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([render_product])

print("Accumulating RTX frames...")
for i in range(120):
    simulation_app.update()

rep.orchestrator.step()
simulation_app.update()

shutil.copy(os.path.join(OUT_DIR, "rgb_0000.png"), OUT_PATH)
shutil.rmtree(OUT_DIR, ignore_errors=True)
print(f"Saved {OUT_PATH}")

simulation_app.close()
