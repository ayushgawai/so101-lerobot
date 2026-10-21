"""One-off diagnostic: opens real-to-sim.usd in headless Isaac Sim (real RTX
renderer) and captures a low, angled view of /World/AWSBuilderCube +
/World/AWSCubePaper to verify the white paper card peeks out from under the
cube on all sides, and shows the handwritten "A" + red dot where visible.

Run with Isaac Sim's own Python (NOT usdenv, which has no Hydra/RTX imaging):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\render_aws_cube_paper_check.py
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
OUT_DIR = tempfile.mkdtemp(prefix="aws_cube_paper_check_")
CAMERA_PATH = "/World/AWSCubePaperCheckCamera"

print(f"Opening {STAGE_PATH}")
open_stage(usd_path=STAGE_PATH)
while is_stage_loading():
    simulation_app.update()
print("Stage loaded")

stage = omni.usd.get_context().get_stage()

bbcache = UsdGeom.BBoxCache(0, ["default", "render"], useExtentsHint=True)
paper_prim = stage.GetPrimAtPath("/World/AWSCubePaper")
cube_prim = stage.GetPrimAtPath("/World/AWSBuilderCube")
rng_p = bbcache.ComputeWorldBound(paper_prim).ComputeAlignedRange()
rng_c = bbcache.ComputeWorldBound(cube_prim).ComputeAlignedRange()
overall_min = [min(rng_p.GetMin()[i], rng_c.GetMin()[i]) for i in range(3)]
overall_max = [max(rng_p.GetMax()[i], rng_c.GetMax()[i]) for i in range(3)]
center = [(overall_min[i] + overall_max[i]) * 0.5 for i in range(3)]
span = max(overall_max[0] - overall_min[0], overall_max[1] - overall_min[1])
print("Combined bbox min:", overall_min, "max:", overall_max)


def shoot(tag, eye, target, out_name, focal_length=24.0):
    quat = look_at_quaternion(np.array(eye), np.array(target)).numpy()
    cam_prim = stage.DefinePrim(CAMERA_PATH, "Camera")
    cam = UsdGeom.Camera(cam_prim)
    cam.CreateFocalLengthAttr(focal_length)
    cam.CreateClippingRangeAttr((0.01, 1000.0))
    xformable = UsdGeom.Xformable(cam_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(tuple(eye))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    )

    render_product = rep.create.render_product(CAMERA_PATH, resolution=(1280, 1280))
    writer = rep.WriterRegistry.get("BasicWriter")
    shot_dir = os.path.join(OUT_DIR, tag)
    os.makedirs(shot_dir, exist_ok=True)
    writer.initialize(output_dir=shot_dir, rgb=True)
    writer.attach([render_product])

    print(f"[{tag}] accumulating RTX frames...")
    for i in range(150):
        simulation_app.update()

    rep.orchestrator.step()
    rep.orchestrator.wait_until_complete()
    src_path = os.path.join(shot_dir, "rgb_0000.png")
    while not os.path.exists(src_path):
        simulation_app.update()

    out_path = os.path.join(
        r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\usd-composer-stages",
        out_name,
    )
    shutil.copy(src_path, out_path)
    print(f"[{tag}] saved {out_path}")


# low, close 3/4 angled view to see the paper peeking out under the cube
angled_eye = [center[0] - span * 1.7, center[1] - span * 1.7, overall_max[2] + span * 1.1]
angled_target = [center[0], center[1], overall_min[2]]
shoot("angled", angled_eye, angled_target, "aws_cube_paper_check_angled.png", focal_length=28.0)

# top-down to see the paper margin evenly on all four sides
top_eye = [center[0], center[1], overall_max[2] + span * 2.2]
top_target = [center[0], center[1], overall_min[2]]
shoot("top", top_eye, top_target, "aws_cube_paper_check_top.png", focal_length=35.0)

shutil.rmtree(OUT_DIR, ignore_errors=True)
simulation_app.close()
