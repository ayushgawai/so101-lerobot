"""MotorsBus.disconnect with a latched motor fault (no hardware needed).

Feetech status packets carry the motor's fault flags, so after an Overload trip every write to
that motor "fails" even though it was applied. Observed on the SO-101 gripper (id 6): the
Torque_Enable=0 write raised "[RxPacketError] Overload error!" yet Torque_Enable read back 0.
"""

from __future__ import annotations

import logging
from unittest.mock import MagicMock

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

COMM_SUCCESS = 0
COMM_RX_TIMEOUT = -3001
OVERLOAD = 32


def _bus(gripper_torque_readback: int | None):
    bus = FeetechMotorsBus(
        port="COM99",
        motors={
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )
    bus.port_handler = MagicMock(is_open=True)

    def disable_torque(motor, num_retry=0):
        if motor == "gripper":
            raise RuntimeError("Failed to write 'Torque_Enable' on id_=6 with '0'. [RxPacketError] Overload error!")

    bus.disable_torque = MagicMock(side_effect=disable_torque)
    if gripper_torque_readback is None:  # motor doesn't answer at all
        bus._read = MagicMock(return_value=(0, COMM_RX_TIMEOUT, 0))
    else:
        bus._read = MagicMock(return_value=(gripper_torque_readback, COMM_SUCCESS, OVERLOAD))
    return bus


def test_latched_fault_with_torque_off_is_reported_as_safe(caplog):
    bus = _bus(gripper_torque_readback=0)

    with caplog.at_level(logging.INFO):
        bus.disconnect(disable_torque=True)

    assert "Failed to disable torque" not in caplog.text
    assert "torque is off" in caplog.text
    assert "Power-cycle" in caplog.text
    bus.port_handler.closePort.assert_called_once()


def test_torque_still_on_is_still_reported_as_failure(caplog):
    bus = _bus(gripper_torque_readback=1)

    with caplog.at_level(logging.INFO):
        bus.disconnect(disable_torque=True)

    assert "Failed to disable torque on motor 'gripper'" in caplog.text
    bus.port_handler.closePort.assert_called_once()


def test_unreadable_motor_is_still_reported_as_failure(caplog):
    bus = _bus(gripper_torque_readback=None)

    with caplog.at_level(logging.INFO):
        bus.disconnect(disable_torque=True)

    assert "Failed to disable torque on motor 'gripper'" in caplog.text
