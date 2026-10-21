"""Isolated test: connect the follower ONLY, via the exact lerobot-sim / Isaac-Sim
python environment, no Kit, no leader -- calling robot.connect() exactly the way
LeRobotSO101Interface.connect() does (self.robot.connect(), no arguments), so
this is a faithful reproduction of the step that died in the full teleop script.
Prints after every step so that if the process dies with no Python traceback
(as happened under the full teleop script), we can see from stdout exactly
which step it got to.

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\isolate_follower_connect.py
"""
import os

# This file lives at <repo_root>/source/sim_to_real_so101/scripts/, so the
# repo root is FOUR directories up, not three -- matches
# leader_arm_teleop_raw_isaacsim.py's _REPO_ROOT_DIR computation exactly
# (_REPO_SOURCE_DIR = 3 dirnames, then _REPO_ROOT_DIR = one more).
_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO_ROOT = os.path.dirname(_REPO_SOURCE_DIR)
_CALIBRATION_DIR = os.path.join(REPO_ROOT, "calibration")
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "robots", "so_follower", "my_so_arm.json")), (
    f"Sanity check failed: expected calibration file not found under {_CALIBRATION_DIR}. "
    "REPO_ROOT path computation is likely wrong again -- fix before proceeding."
)
os.environ["HF_LEROBOT_CALIBRATION"] = _CALIBRATION_DIR

print("step 0: env set, importing lerobot...", flush=True)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402

print("step 1: lerobot imported, building config...", flush=True)
cfg = SO101FollowerConfig(port="COM3", id="my_so_arm", cameras={})

print("step 2: constructing robot object...", flush=True)
robot = SO101Follower(cfg)

print("step 3: calling robot.connect() -- same call LeRobotSO101Interface.connect() makes...", flush=True)
robot.connect()
print("step 4: robot.connect() returned successfully!", flush=True)

print("step 5: calling robot.get_observation()...", flush=True)
obs = robot.get_observation()
print(f"step 6: got observation -> {obs}", flush=True)

print("step 7: disconnecting...", flush=True)
robot.disconnect()
print("step 8: DONE, all steps completed cleanly.", flush=True)
