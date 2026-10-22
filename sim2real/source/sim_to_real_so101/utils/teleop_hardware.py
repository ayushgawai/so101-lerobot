"""SO-101 teleop hardware helpers; no Kit, policy, or GPU dependencies.

LeRobot imports are delayed so calibration checks/tests can run on plain Python.
"""
import json
from pathlib import Path

from .dual_teleop_control import KEYS, checked_action


def validate_calibration(path):
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"Cannot read calibration {path}: {exc}") from exc
    names = [key.removesuffix(".pos") for key in KEYS]
    if not isinstance(data, dict) or set(data) != set(names):
        raise ValueError(f"{path}: expected exactly these six joints: {', '.join(names)}")
    for motor_id, name in enumerate(names, 1):
        row = data[name]
        fields = ("id", "drive_mode", "homing_offset", "range_min", "range_max")
        if not isinstance(row, dict) or any(type(row.get(k)) is not int for k in fields):
            raise ValueError(f"{path}: {name} needs integer fields {fields}")
        if row["id"] != motor_id or row["drive_mode"] != 0:
            raise ValueError(f"{path}: {name} must use ID {motor_id} and drive_mode 0")
        if not -2047 <= row["homing_offset"] <= 2047:
            raise ValueError(f"{path}: invalid homing offset for {name}")
        if not 0 <= row["range_min"] < row["range_max"] <= 4095:
            raise ValueError(f"{path}: invalid encoder range for {name}")
    return data


def connect_calibrated(robot, hold_current=False):
    """Never recalibrate implicitly. Preload a validated hold goal before torque on."""
    try:
        robot.bus.connect()
        if not robot.is_calibrated:
            raise RuntimeError(f"Motor EEPROM differs from {robot.calibration_fpath}; verify/calibrate the arm explicitly")
        if hold_current:
            positions = robot.bus.sync_read("Present_Position")
            checked_action({f"{key}.pos": value for key, value in positions.items()})
            robot.bus.disable_torque()
            robot.bus.sync_write("Goal_Position", positions)
        robot.configure()
    except BaseException:
        if robot.bus.is_connected:
            try:
                robot.bus.disconnect(disable_torque=True)
            except Exception:
                pass  # Preserve the startup exception; caller also attempts cleanup.
        raise


class TeleopSO101Interface:
    """Minimal adapter for the raw controller, with explicitly normalized units."""
    # Preserve the project's existing real-to-USD mapping, in joint order.
    RANGES = ((-110, 110), (-100, 100), (-100, 90), (-95, 95), (-160, 160), (-10, 100))

    def __init__(self, device, port, id, cameras, fps, kind="leader"):
        self.port, self.id, self.kind = port, id, kind
        if cameras:
            raise ValueError("Teleop motor interfaces use separate optional camera previews")

    def init_device(self):
        import os
        root = Path(os.environ["HF_LEROBOT_CALIBRATION"])
        if self.kind == "leader":
            from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
            self.robot = SO101Leader(SO101LeaderConfig(
                port=self.port, id=self.id, use_degrees=False,
                calibration_dir=root / "teleoperators" / "so_leader"))
        elif self.kind == "follower":
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            self.robot = SO101Follower(SO101FollowerConfig(
                port=self.port, id=self.id, use_degrees=False, cameras={},
                calibration_dir=root / "robots" / "so_follower",
                disable_torque_on_disconnect=True))
        else:
            raise ValueError(f"Unknown arm kind: {self.kind}")

    def connect(self):
        connect_calibrated(self.robot, hold_current=self.kind == "follower")

    def mapped_degrees(self, action):
        action = checked_action(action)
        result = []
        for index, (key, (lower, upper)) in enumerate(zip(KEYS, self.RANGES)):
            fraction = action[key] / 100 if index == 5 else (action[key] + 100) / 200
            result.append(lower + fraction * (upper - lower))
        return result
