"""One-off diagnostic: opens real-to-sim.usd in headless Isaac Sim (real RTX
renderer) and captures a view showing the restyled walls (Cube->Mesh + new
diffuse/roughness/normal texture set), so the new look can be checked against
the requested art direction before calling this done.

Deliberately NOT calling timeline.play(): this stage's root_joint has a
disjointed-body-transform PhysX warning on load, and physics snaps/jitters
the whole articulation away from its authored rest pose over the frames we'd
be accumulating. Render with physics stopped, matching render_floor_check.py.

Run with Isaac Sim's own Python (NOT usdenv, which has no Hydra/RTX imaging):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\render_wall_check.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import os
import shutil
import tempfile

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.core.experimental.utils.transform import look_at_quaternion
from pxr import Gf, UsdGeom
import omni.usd

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)
OUT_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop"
    r"\usd-composer-stages\wall_material_check_full.png"
)
OUT_DIR = tempfile.mkdtemp(prefix="wall_check_")
CAMERA_PATH = "/World/WallCheckCamera"

print(f"Opening {STAGE_PATH}")
open_stage(usd_path=STAGE_PATH)
while is_stage_loading():
    simulation_app.update()
print("Stage loaded")

stage = omni.usd.get_context().get_stage()

# centered, far back, facing the North wall square-on floor-to-ceiling so the
# full vertical gradient is visible in one shot.
eye = np.array([0.0, -1.5, 1.25])
target = np.array([0.0, 2.0, 1.25])
quat = look_at_quaternion(eye, target).numpy()  # (w, x, y, z)

cam_prim = stage.DefinePrim(CAMERA_PATH, "Camera")
cam = UsdGeom.Camera(cam_prim)
cam.CreateFocalLengthAttr(24.0)
cam.CreateClippingRangeAttr((0.01, 1000.0))
xformable = UsdGeom.Xformable(cam_prim)
xformable.ClearXformOpOrder()
xformable.AddTranslateOp().Set(tuple(eye))
xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
    Gf.Quatd(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
)

render_product = rep.create.render_product(CAMERA_PATH, resolution=(960, 960))
writer = rep.WriterRegistry.get("BasicWriter")
os.makedirs(OUT_DIR, exist_ok=True)
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([render_product])

print("Accumulating RTX frames (physics stopped)...")
for i in range(150):
    simulation_app.update()
    if (i + 1) % 30 == 0:
        print(f"  frame {i + 1}/150")

rep.orchestrator.step()
rep.orchestrator.wait_until_complete()
src_path = os.path.join(OUT_DIR, "rgb_0000.png")
while not os.path.exists(src_path):
    simulation_app.update()

shutil.copy(src_path, OUT_PATH)
shutil.rmtree(OUT_DIR, ignore_errors=True)
print(f"Saved {OUT_PATH}")

simulation_app.close()
