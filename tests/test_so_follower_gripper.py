"""Gripper stall relief in SOFollower.send_action (no hardware needed).

Measured on the SO-101 follower gripper (cube held, 50 % torque limit): commanding the goal 50 ticks past
the contact point gives ~17 % load, >= 200 ticks saturates at 50 % and, held ~2 s, trips the latched
Overload fault. Closing = decreasing position.
"""

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock, patch

import pytest

from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

RANGE_TICKS = 2365  # this arm's gripper calibration: 1123..3488
SQUEEZE_UNITS = 50 * 100 / RANGE_TICKS  # 50 ticks in 0-100 units


class FakeArm:
    def __init__(self, tmp_path):
        self.robot = SO101Follower(SO101FollowerConfig(port="COM99", id="test_arm", calibration_dir=tmp_path))
        self.robot.calibration = {
            "gripper": MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=1123, range_max=3488)
        }
        self.robot.cameras = {}
        self.gripper_present = 50.0
        self.now = 0.0
        self.robot.bus.sync_read = MagicMock(side_effect=lambda reg, motors=None, **kw: {"gripper": self.gripper_present})
        self.robot.bus.sync_write = MagicMock()

    def send_gripper(self, goal: float, *, dt: float = 1 / 30) -> float:
        """Advance time by one control step, send a gripper goal, return the goal actually written."""
        self.now += dt
        with (
            patch.object(type(self.robot.bus), "is_connected", new_callable=PropertyMock, return_value=True),
            patch("lerobot.robots.so_follower.so_follower.time.perf_counter", return_value=self.now),
        ):
            sent = self.robot.send_action({"gripper.pos": goal})
        return sent["gripper.pos"]


def test_free_closing_motion_is_not_capped(tmp_path):
    arm = FakeArm(tmp_path)
    for step in range(60):  # 2 s of the gripper actually moving closed
        arm.gripper_present = 50.0 - step
        assert arm.send_gripper(0.0) == 0.0


def test_blocked_gripper_is_capped_after_stall_time(tmp_path):
    arm = FakeArm(tmp_path)
    arm.gripper_present = 30.0  # jaws stopped on the cube; leader fully closed

    early = [arm.send_gripper(0.0) for _ in range(10)]  # ~0.33 s blocked
    assert all(g == 0.0 for g in early), "must not cap before the stall time (would slow normal motion starts)"

    late = [arm.send_gripper(0.0) for _ in range(30)]  # up to ~1.3 s blocked, still < 2 s overload timer
    assert late[-1] == pytest.approx(30.0 - SQUEEZE_UNITS)


def test_opening_releases_the_cap_immediately(tmp_path):
    arm = FakeArm(tmp_path)
    arm.gripper_present = 30.0
    for _ in range(30):
        arm.send_gripper(0.0)
    assert arm.send_gripper(0.0) == pytest.approx(30.0 - SQUEEZE_UNITS)

    assert arm.send_gripper(80.0) == 80.0  # leader opens


def test_other_joints_are_untouched(tmp_path):
    arm = FakeArm(tmp_path)
    arm.gripper_present = 30.0
    for _ in range(30):
        arm.send_gripper(0.0)
    with patch.object(type(arm.robot.bus), "is_connected", new_callable=PropertyMock, return_value=True):
        sent = arm.robot.send_action({"wrist_roll.pos": 12.5, "gripper.pos": 0.0})
    assert sent["wrist_roll.pos"] == 12.5
