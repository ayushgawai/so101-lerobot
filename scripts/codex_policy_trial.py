"""Run the normal LeRobot rollout with camera snapshots and guarded startup."""
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault('HF_HUB_OFFLINE', '1')
os.environ.setdefault('HF_DATASETS_OFFLINE', '1')
os.environ.setdefault('PYTHONUTF8', '1')

import cv2
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.scripts.lerobot_rollout import main

OUT = Path('.cache/codex_policy_trial')
OUT.mkdir(parents=True, exist_ok=True)
CONNECTED = []
original_connect = SOFollower.connect
original_observe = SOFollower.get_observation
original_action = SOFollower.send_action

def connect(self, calibrate=True):
    # Motor power may have been cycled: latch the measured pose before enabling torque.
    self.bus.connect()
    try:
        if not self.bus.is_calibrated:
            raise RuntimeError('Calibration mismatch; refusing trial')
        present = self.bus.sync_read('Present_Position', num_retry=2)
        self.bus.sync_write('Goal_Position', present)
    finally:
        self.bus.disconnect(disable_torque=False)
    original_connect(self, calibrate=False)
    CONNECTED.append(self)
    self._trial_frame_time = 0
    self._trial_monitor_time = 0
    self._trial_count = 0
    self._trial_previous_goal = None

def observe(self):
    obs = original_observe(self)
    self._trial_position = {k: v for k, v in obs.items() if k.endswith('.pos')}
    now = time.monotonic()
    if now - self._trial_frame_time >= .5:
        self._trial_frame_time = now
        self._trial_count += 1
        for key in ('top_cam', 'gripper_cam'):
            if key in obs:
                bgr = cv2.cvtColor(obs[key], cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(OUT / f'{key}_latest.jpg'), bgr)
                cv2.imwrite(str(OUT / f'{key}_{self._trial_count:04d}.jpg'), bgr)
        (OUT / 'latest.json').write_text(json.dumps({'frame': self._trial_count, 'position': self._trial_position}))
    return obs

def action(self, values):
    if (OUT / 'STOP').exists():
        raise KeyboardInterrupt('Camera supervisor requested stop')
    now = time.monotonic()
    if now - self._trial_monitor_time >= .25:
        self._trial_monitor_time = now
        status = self.bus.sync_read('Status', normalize=False, num_retry=2)
        temps = self.bus.sync_read('Present_Temperature', normalize=False, num_retry=2)
        loads = self.bus.sync_read('Present_Load', normalize=False, num_retry=2)
        if any(status.values()) or max(temps.values()) >= 55 or any(abs(v) > (400 if k == 'gripper' else 850) for k, v in loads.items()):
            raise RuntimeError(f'Motor monitor stopped trial: status={status}, temperature={temps}, load={loads}')
    present = self._trial_position
    if self._trial_previous_goal:
        lag = {k: present[k] - v for k, v in self._trial_previous_goal.items() if k != 'gripper.pos'}
        if any(abs(v) > 8 for v in lag.values()):
            raise RuntimeError(f'Motor did not follow prior command: {lag}')
    bounded = {}
    for key, val in values.items():
        if not math.isfinite(val):
            raise RuntimeError('Non-finite policy action')
        name = key.removesuffix('.pos')
        cal = self.calibration[name]
        limit = (cal.range_max - cal.range_min) * 180 / 4095
        lo, hi = (0, 100) if name == 'gripper' else (-limit, limit)
        bounded[key] = max(lo, min(hi, float(val)))
    sent = original_action(self, bounded)
    self._trial_previous_goal = sent
    return sent

SOFollower.connect = connect
SOFollower.get_observation = observe
SOFollower.send_action = action
try:
    main()
finally:
    for robot in CONNECTED:
        if robot.bus.is_connected:
            try:
                position = robot.bus.sync_read('Present_Position', num_retry=2)
                robot.bus.sync_write('Goal_Position', {k: v for k, v in position.items() if k != 'gripper'})
            finally:
                robot.disconnect()
