"""Isolated test: boot Isaac Sim (Kit) with NO stage, NO physics, NO robot rig
-- just SimulationApp() -- then connect both leader (COM4) and follower (COM3)
exactly as isolate_dual_arm_connect.py did (which passed cleanly with no Kit
involved at all).

Purpose: isolate_follower_connect.py proved the follower alone is fine.
isolate_dual_arm_connect.py proved both arms together are fine, with no Kit.
The full leader_arm_teleop_raw_isaacsim.py run died right at follower.connect()
with Kit fully booted AND a busy scene (USD stage loaded, physics playing,
camera sensors ticking). This script splits those two remaining variables:
does Kit's mere presence (render thread, GPU context, disabled signal
handlers -- see --/app/installSignalHandlers=0 in the real script's Kit args)
break dual-arm connect on its own, independent of scene complexity?

Prints after every step so a silent death (no traceback) still shows exactly
which step it reached.

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\isolate_kit_dual_arm_connect.py
"""
import os

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO_ROOT = os.path.dirname(_REPO_SOURCE_DIR)
_CALIBRATION_DIR = os.path.join(REPO_ROOT, "calibration")
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "robots", "so_follower", "my_so_arm.json")), (
    f"Sanity check failed: expected calibration file not found under {_CALIBRATION_DIR}."
)
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "teleoperators", "so_leader", "my_so_arm.json")), (
    f"Sanity check failed: expected leader calibration file not found under {_CALIBRATION_DIR}."
)
os.environ["HF_LEROBOT_CALIBRATION"] = _CALIBRATION_DIR

print("step 0: env set. Booting Kit (SimulationApp) -- same non-headless mode as the real script...", flush=True)
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": False})
print("step 1: Kit booted, app ready. Importing lerobot...", flush=True)

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

print("step 2: lerobot imported, building configs...", flush=True)
leader_cfg = SO101LeaderConfig(port="COM4", id="my_so_arm")
follower_cfg = SO101FollowerConfig(port="COM3", id="my_so_arm", cameras={})

print("step 3: constructing leader + follower objects...", flush=True)
leader = SO101Leader(leader_cfg)
follower = SO101Follower(follower_cfg)

print("step 4: simulation_app.update() (one tick, matching the real script's usage pattern)...", flush=True)
simulation_app.update()

print("step 5: connecting leader (COM4)...", flush=True)
leader.connect()
print("step 6: leader connected OK", flush=True)

print("step 7: connecting follower (COM3) -- this is the point that died before...", flush=True)
follower.connect()
print("step 8: follower connected OK", flush=True)

print("step 9: reading leader action...", flush=True)
action = leader.get_action()
print(f"step 10: got leader action -> {action}", flush=True)

print("step 11: sending leader action to follower...", flush=True)
follower.send_action(action)
print("step 12: send_action OK", flush=True)

print("step 13: a few simulation_app.update() ticks (matching the real script's main loop shape)...", flush=True)
for i in range(10):
    action = leader.get_action()
    follower.send_action(action)
    simulation_app.update()
print("step 14: 10 ticks completed OK", flush=True)

print("step 15: disconnecting both...", flush=True)
leader.disconnect()
follower.disconnect()
print("step 16: DONE, all steps completed cleanly.", flush=True)

simulation_app.close()
