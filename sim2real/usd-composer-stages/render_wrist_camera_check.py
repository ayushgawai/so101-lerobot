"""One-off diagnostic: opens real-to-sim.usd in a headless Isaac Sim (real
RTX renderer) and captures a frame through the newly-authored
/World/SO_ARM101_USD/gripper/WristCamera -- seated at the camera_mount
bracket geometry -- to visually confirm it's aimed at the gripper's own
workspace (jaws / table in front) rather than into the robot body or off
into empty space.

Run with Isaac Sim's own Python (NOT usdenv, which has no Hydra/RTX imaging):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\render_wrist_camera_check.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import os
import shutil
import tempfile

import omni.replicator.core as rep
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)
OUT_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop"
    r"\usd-composer-stages\wrist_camera_pov_check.png"
)
OUT_DIR = tempfile.mkdtemp(prefix="wrist_cam_check_")
WRIST_CAMERA_PATH = "/World/SO_ARM101_USD/gripper/WristCamera"

print(f"Opening {STAGE_PATH}")
open_stage(usd_path=STAGE_PATH)
while is_stage_loading():
    simulation_app.update()
print("Stage loaded")

# Deliberately NOT calling timeline.play(): this stage's root_joint has a
# disjointed-body-transform PhysX warning on load, and physics snaps/jitters
# the whole articulation away from its authored rest pose over the frames
# we'd be accumulating. We want the authored/static geometry, so render with
# physics stopped.

render_product = rep.create.render_product(WRIST_CAMERA_PATH, resolution=(960, 720))
writer = rep.WriterRegistry.get("BasicWriter")
os.makedirs(OUT_DIR, exist_ok=True)
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([render_product])

print("Accumulating RTX frames...")
for i in range(120):
    simulation_app.update()
    if (i + 1) % 30 == 0:
        print(f"  frame {i + 1}/120")

rep.orchestrator.step()
simulation_app.update()

shutil.copy(os.path.join(OUT_DIR, "rgb_0000.png"), OUT_PATH)
shutil.rmtree(OUT_DIR, ignore_errors=True)
print(f"Saved {OUT_PATH}")

simulation_app.close()
