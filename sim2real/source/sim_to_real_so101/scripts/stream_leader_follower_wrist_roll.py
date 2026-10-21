"""Stream leader/follower wrist_roll (and all joints) in degrees, side by side,
with NO Kit/Isaac Sim involved -- same proven-stable pattern as
isolate_dual_arm_connect.py (dual-arm connect with no Kit has passed cleanly
every time it's been tried this session).

Purpose: get the real-follower-vs-leader-command degree relationship at a
specific, physically-describable reference pose (e.g. "jaws open sideways"),
without depending on compare_real_vs_sim_joints.py, which is hitting a
separate, not-yet-understood crash when isaacsim.core.experimental.prims is
imported.

Run with: C:\\Isaac-Sim\\python.bat source\\sim_to_real_so101\\scripts\\stream_leader_follower_wrist_roll.py
Ctrl+C to stop.
"""
import os
import time

_REPO_SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO_ROOT = os.path.dirname(_REPO_SOURCE_DIR)
_CALIBRATION_DIR = os.path.join(REPO_ROOT, "calibration")
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "robots", "so_follower", "my_so_arm.json"))
assert os.path.isfile(os.path.join(_CALIBRATION_DIR, "teleoperators", "so_leader", "my_so_arm.json"))
os.environ["HF_LEROBOT_CALIBRATION"] = _CALIBRATION_DIR

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

leader_cfg = SO101LeaderConfig(port="COM4", id="my_so_arm")
follower_cfg = SO101FollowerConfig(port="COM3", id="my_so_arm", cameras={})
leader = SO101Leader(leader_cfg)
follower = SO101Follower(follower_cfg)

print("connecting leader (COM4)...", flush=True)
leader.connect()
print("connecting follower (COM3)...", flush=True)
follower.connect()
print("connected. streaming leader (cmd) vs follower (real) degrees. Ctrl+C to stop.\n", flush=True)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

try:
    while True:
        action = leader.get_action()
        obs = follower.get_observation()
        follower.send_action(action)
        line = "  ".join(
            f"{j}: cmd={action[j + '.pos']:+7.2f} real={obs[j + '.pos']:+7.2f} "
            f"diff={obs[j + '.pos'] - action[j + '.pos']:+7.2f}"
            for j in JOINTS
        )
        print(line, flush=True)
        time.sleep(0.3)
except KeyboardInterrupt:
    print("\nstopped.")
finally:
    leader.disconnect()
    follower.disconnect()
