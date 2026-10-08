"""SOFollower.connect: the recovery hint must match the stage that failed (no hardware needed)."""

from __future__ import annotations

import logging
from unittest.mock import MagicMock, PropertyMock, patch

import pytest

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def _robot(tmp_path):
    robot = SO101Follower(SO101FollowerConfig(port="COM99", id="test_arm", calibration_dir=tmp_path))
    robot.bus.disconnect = MagicMock()
    return robot


def _fake_bus_connect(port: dict, *, opens_port: bool, error: Exception | None = None):
    """bus.connect stand-in: optionally opens the port, then optionally fails (like a handshake error)."""

    def connect():
        port["open"] = opens_port
        if error is not None:
            raise error

    return MagicMock(side_effect=connect)


def _connect_and_get_log(robot, caplog, port: dict) -> str:
    bus_cls = type(robot.bus)
    with (
        patch.object(bus_cls, "is_connected", new_callable=PropertyMock, side_effect=lambda: port["open"]),
        patch.object(bus_cls, "is_calibrated", new_callable=PropertyMock, return_value=True),
        caplog.at_level(logging.ERROR),
        pytest.raises(Exception),
    ):
        robot.connect(calibrate=False)
    return caplog.text


def test_port_failure_gives_port_hint_not_motor_steps(tmp_path, caplog):
    robot = _robot(tmp_path)
    port = {"open": False}
    robot.bus.connect = _fake_bus_connect(port, opens_port=False, error=ConnectionError("COM99"))

    log = _connect_and_get_log(robot, caplog, port)

    assert "lerobot-find-port" in log
    assert "Power-cycle" not in log


def test_motor_failure_gives_motor_steps_pointing_below(tmp_path, caplog):
    robot = _robot(tmp_path)
    port = {"open": False}
    robot.bus.connect = _fake_bus_connect(port, opens_port=True, error=RuntimeError("Missing motor IDs: 5"))

    log = _connect_and_get_log(robot, caplog, port)

    assert "Power-cycle" in log
    assert "error below" in log
    robot.bus.disconnect.assert_called_once()  # port opened, so it must be released


def test_camera_failure_gives_camera_hint(tmp_path, caplog):
    robot = _robot(tmp_path)
    port = {"open": False}
    robot.bus.connect = _fake_bus_connect(port, opens_port=True)
    cam = MagicMock(is_connected=False)
    cam.connect.side_effect = ConnectionError("OpenCVCamera(1) failed to open")
    robot.cameras = {"top_cam": cam}

    log = _connect_and_get_log(robot, caplog, port)

    assert "lerobot-find-cameras" in log
    assert "Power-cycle" not in log
