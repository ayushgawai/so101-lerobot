"""One-off diagnostic: opens room-and-table-with-aws-cube.usd in a headless
Isaac Sim (real RTX renderer), points a camera close at the AWS Builder cube
from roughly the angle the user's screenshot showed, lets RTX accumulate for
many frames (shadows/AO need several frames to converge in real-time mode),
and saves an RGB capture to disk -- so the "dark shadow curve" question can
be answered from an actual render instead of guessing.

Run with Isaac Sim's own Python (NOT usdenv, which has no Hydra/RTX imaging):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\rtx_shadow_check.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import os

import numpy as np
import omni.timeline
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.core.experimental.utils.transform import look_at_quaternion
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from PIL import Image

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\room-and-table-with-aws-cube.usd"
)
OUT_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop"
    r"\usd-composer-stages\aws_cube_rtx_check_final.png"
)

print(f"Opening {STAGE_PATH}")
open_stage(usd_path=STAGE_PATH)
while is_stage_loading():
    simulation_app.update()
print("Stage loaded")

# Cube center is at world (0, 0, 0.775) per build_aws_builder_cube.py
# diagnostics. Close, elevated, angled view similar to a user orbiting in
# close to inspect the object -- matches the framing in the reported
# screenshot (cube fills most of frame, tabletop visible just below it).
eye = np.array([0.55, -0.65, 1.00])
target = np.array([0.0, 0.0, 0.755])
quat = look_at_quaternion(eye, target).numpy()

cam = RtxCamera("/World/CheckCamera", tick_rate=30.0, translations=eye, orientations=quat)
cam.camera.set_focal_lengths(24.0)
cam.camera.set_clipping_ranges(0.01, 1000.0)

resolution = (720, 720)
sensor = CameraSensor(cam, resolution=resolution, annotators=["rgb"])

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Accumulating RTX frames...")
rgb_data = None
for i in range(180):
    simulation_app.update()
    data, info = sensor.get_data("rgb")
    if data is not None:
        rgb_data = data
    if (i + 1) % 30 == 0:
        print(f"  frame {i + 1}/180, have_rgb={rgb_data is not None}")

if rgb_data is not None:
    arr = rgb_data.numpy()
    print("RGB array shape:", arr.shape, "dtype:", arr.dtype)
    if arr.shape[-1] == 4:
        arr = arr[:, :, :3]
    img = Image.fromarray(arr.astype(np.uint8))
    img.save(OUT_PATH)
    print(f"SAVED {OUT_PATH}")
else:
    print("NO RGB DATA CAPTURED")

timeline.stop()
simulation_app.close()
