"""Small, bounded, visually supervised SO101 steps; never auto-home or recalibrate."""
import argparse
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.cache/codex_pick_place'
OUT.mkdir(parents=True, exist_ok=True)
CAL = json.loads((ROOT / 'calibration/robots/so_follower/my_so_arm.json').read_text())
bus = FeetechMotorsBus(port='COM3', motors={
    name: Motor(v['id'], 'sts3215', MotorNormMode.RANGE_0_100 if name == 'gripper' else MotorNormMode.DEGREES)
    for name, v in CAL.items()
}, calibration={k: MotorCalibration(**v) for k, v in CAL.items()})

def read(name):
    return bus.sync_read(name, normalize=name in ('Present_Position', 'Goal_Position'), num_retry=2)

def state():
    return {field: read(field) for field in (
        'Present_Position', 'Goal_Position', 'Present_Load', 'Present_Temperature', 'Torque_Enable', 'Operating_Mode', 'P_Coefficient', 'Torque_Limit', 'Status')}

def snapshot(label):
    import cv2
    paths = []
    for index in (1, 0):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        try:
            if not cap.isOpened():
                raise RuntimeError(f'Camera {index} unavailable')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            for _ in range(10):
                ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f'No frame from camera {index}')
            path = OUT / f'{label}_camera_{index}.jpg'
            cv2.imwrite(str(path), frame)
            paths.append(str(path))
        finally:
            cap.release()
    return paths

parser = argparse.ArgumentParser()
parser.add_argument('--target', help='Comma separated joint=value; body degrees, gripper 0..100')
parser.add_argument('--seconds', type=float, default=3)
parser.add_argument('--label', default='state')
parser.add_argument('--snapshot', action='store_true')
args = parser.parse_args()
bus.connect()
moving = False
try:
    actual_cal = {k: asdict(v) for k, v in bus.read_calibration().items()}
    before = state()
    result = {'before': before, 'calibration_matches': actual_cal == CAL, 'actual_calibration': actual_cal}
    print(json.dumps({'before': before, 'calibration_matches': actual_cal == CAL}), flush=True)
    if args.target:
        if actual_cal != CAL:
            raise RuntimeError('Calibration mismatch; refusing motion')
        if max(before['Present_Temperature'].values()) >= 55:
            raise RuntimeError('Motor temperature exceeds conservative threshold')
        if any(x != 0 for x in before['Operating_Mode'].values()):
            raise RuntimeError('Not all motors in position mode')
        target = {k: float(v) for k, v in (item.split('=') for item in args.target.split(','))}
        start = before['Present_Position']
        if set(target) - set(start) or not target:
            raise ValueError('Invalid joints')
        for k, v in target.items():
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError('Invalid goal')
            if abs(v - start[k]) > (25 if k == 'gripper' else 35 if k == 'wrist_roll' else 16):
                raise ValueError(f'Too large a single step: {k}')
            raw = bus._unnormalize({CAL[k]['id']: v})[CAL[k]['id']]
            if not CAL[k]['range_min'] <= raw <= CAL[k]['range_max']:
                raise ValueError(f'Outside calibrated limits: {k}')
        if not 2 <= args.seconds <= 10:
            raise ValueError('Duration must be 2..10 seconds')
        # Preserve targets of already holding joints, avoiding cumulative gravity sag.
        held = read('Goal_Position')
        for k, enabled in before['Torque_Enable'].items():
            if not enabled:
                held[k] = start[k]
        bus.sync_write('Goal_Position', held)
        for k, enabled in before['Torque_Enable'].items():
            if not enabled:
                bus.enable_torque(k)
        moving = True
        steps = math.ceil(args.seconds * 25)
        last_goal = dict(held)
        last_goal.update({k: start[k] for k in target})
        for i in range(1, steps + 1):
            tick = time.monotonic()
            present = read('Present_Position')
            lag = {k: present[k] - last_goal[k] for k in target if k != 'gripper'}
            if any(abs(v) > 7 for v in lag.values()):
                raise RuntimeError(f'Position tracking error; stopping at measured pose: {lag}')
            loads = read('Present_Load')
            if any(abs(v) > (300 if k == 'gripper' else 800) for k, v in loads.items()):
                raise RuntimeError('Load threshold exceeded; stopping at measured pose')
            alpha = (1 - math.cos(math.pi * i / steps)) / 2
            goal = dict(held)
            goal.update({k: start[k] + alpha * (v - start[k]) for k, v in target.items()})
            # Limit grip squeeze to 35 raw encoder ticks beyond contact at every step.
            squeeze = 35 * 100 / (CAL['gripper']['range_max'] - CAL['gripper']['range_min'])
            if 'gripper' in target:
                goal['gripper'] = max(goal['gripper'], present['gripper'] - squeeze)
            bus.sync_write('Goal_Position', goal)
            last_goal = goal
            time.sleep(max(0, 1/25 - (time.monotonic() - tick)))
        moving = False
        time.sleep(0.3)
        result['after'] = state()
        print(json.dumps({'after': result['after']}), flush=True)
    if args.snapshot:
        result['images'] = snapshot(args.label)
        print(json.dumps({'images': result['images']}), flush=True)
    (OUT / f'{args.label}.json').write_text(json.dumps(result, indent=2))
except BaseException:
    if moving:
        try:
            bus.sync_write('Goal_Position', read('Present_Position'))
        except Exception:
            pass
    print(json.dumps({'stopped_state': state()}), flush=True)
    if args.snapshot:
        print(json.dumps({'images': snapshot(args.label + '_stopped')}), flush=True)
    raise
finally:
    # Preserve holding torque; disconnecting must not make the arm fall.
    bus.disconnect(disable_torque=False)
