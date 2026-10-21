"""Root-cause diagnostic for the silent (no-traceback) crash that keeps killing
leader_arm_teleop_raw_isaacsim.py right around follower.connect().

What's been ruled out so far (each passed cleanly, multiple times):
  - follower alone, no Kit at all           (isolate_follower_connect.py)
  - both arms, no Kit at all                (isolate_dual_arm_connect.py)
  - both arms, Kit booted but NO stage/physics (isolate_kit_dual_arm_connect.py)
  - Kit + stage + physics, NO hardware at all  (sweep_wrist_roll_screenshots.py)
Never isolated: Kit + stage + physics playing, WITH both arms connecting --
every real test of that exact combination has been inside the full teleop
script, with a lot of other code around it. This reproduces just that.

Two extra layers of instrumentation neither of the earlier scripts had:
  1. `faulthandler.enable()`, writing to a dedicated log file. This catches
     fatal native signals (SIGSEGV/SIGABRT/etc.) and dumps a C+Python stack
     trace even when the failure never becomes a normal Python exception --
     relevant because the existing `except Exception` around connect() in
     the real script already isn't catching this, which means it's probably
     not a normal Python-raised exception at all.
  2. Fine-grained print+flush at every sub-step INSIDE follower.connect()
     (bus.connect() / is_calibrated check / configure()), not just "before"
     and "after" the whole call -- so if it dies mid-connect, we see exactly
     which register write it was on.
  3. An atexit hook: fires on any clean Python-level exit (sys.exit(),
     normal interpreter shutdown) but NOT on a hard process kill /
     TerminateProcess. If it's silent, that alone is a real data point.

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\isolate_busy_scene_dual_arm.py
Check the faulthandler log afterward regardless of outcome -- path is
printed at startup.
"""
import atexit
import faulthandler
import os
import sys

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO_ROOT = os.path.dirname(_REPO_SOURCE_DIR)
_CALIBRATION_DIR = os.path.join(REPO_ROOT, "calibration")
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "robots", "so_follower", "my_so_arm.json"))
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "teleoperators", "so_leader", "my_so_arm.json"))
os.environ["HF_LEROBOT_CALIBRATION"] = _CALIBRATION_DIR

_FAULT_LOG_PATH = os.path.join(os.environ.get("TEMP", REPO_ROOT), "isolate_busy_scene_faulthandler.log")
_fault_log = open(_FAULT_LOG_PATH, "w")
faulthandler.enable(file=_fault_log, all_threads=True)
print(f"step -1: faulthandler enabled, writing to {_FAULT_LOG_PATH}", flush=True)

atexit.register(lambda: print("step ATEXIT: clean Python shutdown reached.", flush=True))

print("step 0: importing isaacsim...", flush=True)
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": False})
print("step 1: Kit booted, app ready.", flush=True)

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf  # noqa: E402

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

print("step 2: lerobot imported.", flush=True)

ROBOT_PRIM_PATH = "/World/SO_ARM101_USD"
REAL_TO_SIM_USD = os.path.join(_REPO_SOURCE_DIR, "sim_to_real_so101", "demo", "real-to-sim.usd")

usd_context = omni.usd.get_context()
usd_context.open_stage(REAL_TO_SIM_USD)
simulation_app.update()
stage = usd_context.get_stage()
print("step 3: stage loaded.", flush=True)

root_joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/root_joint")
local_rot1 = root_joint_prim.GetAttribute("physics:localRot1").Get()
root_joint_prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.0, 0.3, 0.72))
root_joint_prim.GetAttribute("physics:localRot0").Set(local_rot1)
print("step 4: root_joint mount patched.", flush=True)

JOINT_GAINS = {
    "Rotation": dict(stiffness=55, damping=0.7, effort_limit=30),
    "Pitch": dict(stiffness=30, damping=0.8, effort_limit=30),
    "Elbow": dict(stiffness=25, damping=0.7, effort_limit=30),
    "Wrist_Pitch": dict(stiffness=12, damping=0.5, effort_limit=30),
    "Wrist_Roll": dict(stiffness=7, damping=0.5, effort_limit=30),
    "Jaw": dict(stiffness=4, damping=0.3, effort_limit=3),
}
for usd_joint_name, gains in JOINT_GAINS.items():
    joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{usd_joint_name}")
    joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(float(gains["stiffness"]))
    joint_prim.GetAttribute("drive:angular:physics:damping").Set(float(gains["damping"]))
    joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(float(gains["effort_limit"]))
print("step 5: joint gains applied.", flush=True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(30):
    simulation_app.update()
print("step 6: physics playing, 30 settle ticks done -- scene is now exactly as busy as the real script's connect point.", flush=True)

# --- leader connect ---
leader_cfg = SO101LeaderConfig(port="COM4", id="my_so_arm")
leader = SO101Leader(leader_cfg)
print("step 7: leader object constructed.", flush=True)
leader.bus.connect()
print("step 8: leader bus.connect() OK.", flush=True)
print(f"step 9: leader is_calibrated = {leader.is_calibrated}", flush=True)
if not leader.is_calibrated:
    leader.calibrate()
    print("step 9b: leader.calibrate() returned (unexpected -- should have been calibrated).", flush=True)
leader.configure()
print("step 10: leader.configure() OK.", flush=True)

# --- follower connect, broken into the SAME sub-steps SO101Follower.connect()
# performs internally, so a death here pinpoints which one ---
follower_cfg = SO101FollowerConfig(port="COM3", id="my_so_arm", cameras={})
follower = SO101Follower(follower_cfg)
print("step 11: follower object constructed.", flush=True)
follower.bus.connect()
print("step 12: follower bus.connect() OK.", flush=True)
print(f"step 13: follower is_calibrated = {follower.is_calibrated}", flush=True)
if not follower.is_calibrated:
    print("step 13b: WARNING -- follower not calibrated, this would trigger interactive calibrate() in the real flow.", flush=True)
else:
    print("step 14: about to call follower.configure() -- this is the ~42-write burst that has been dying...", flush=True)
    follower.configure()
    print("step 15: follower.configure() OK -- the burst survived.", flush=True)

print("step 16: reading follower observation...", flush=True)
obs = follower.get_observation()
print(f"step 17: got observation -> {obs}", flush=True)

print("step 18: disconnecting...", flush=True)
leader.disconnect()
follower.disconnect()
print("step 19: DONE, all steps completed cleanly.", flush=True)

timeline.stop()
simulation_app.close()
_fault_log.close()
