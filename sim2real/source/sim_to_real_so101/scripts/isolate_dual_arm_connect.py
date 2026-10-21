"""Isolated test: connect BOTH leader (COM4) and follower (COM3) simultaneously,
via the exact lerobot-sim / Isaac-Sim python environment, but with NO Kit/Isaac
Sim involved at all. Tests whether two simultaneous serial connections alone
(independent of Isaac Sim) can reproduce the silent crash seen in the full
leader_arm_teleop_raw_isaacsim.py run -- isolate_follower_connect.py already
proved the follower alone is fine; this adds the leader back in.

Prints after every step so that if the process dies with no Python traceback,
we can see from stdout exactly which step it got to.

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\isolate_dual_arm_connect.py
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

print("step 0: env set, importing lerobot...", flush=True)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

print("step 1: lerobot imported, building configs...", flush=True)
leader_cfg = SO101LeaderConfig(port="COM4", id="my_so_arm")
follower_cfg = SO101FollowerConfig(port="COM3", id="my_so_arm", cameras={})

print("step 2: constructing leader object...", flush=True)
leader = SO101Leader(leader_cfg)

print("step 3: constructing follower object...", flush=True)
follower = SO101Follower(follower_cfg)

print("step 4: connecting leader (COM4)...", flush=True)
leader.connect()
print("step 5: leader connected OK", flush=True)

print("step 6: connecting follower (COM3) -- this is the point that died before...", flush=True)
follower.connect()
print("step 7: follower connected OK", flush=True)

print("step 8: reading leader action...", flush=True)
action = leader.get_action()
print(f"step 9: got leader action -> {action}", flush=True)

print("step 10: reading follower observation...", flush=True)
obs = follower.get_observation()
print(f"step 11: got follower observation -> {obs}", flush=True)

print("step 12: sending leader action to follower (one send_action call)...", flush=True)
follower.send_action(action)
print("step 13: send_action OK", flush=True)

print("step 14: disconnecting both...", flush=True)
leader.disconnect()
follower.disconnect()
print("step 15: DONE, all steps completed cleanly.", flush=True)
