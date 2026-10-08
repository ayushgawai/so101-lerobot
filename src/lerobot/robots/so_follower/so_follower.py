#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_so_follower import SO100FollowerRobotConfig, SO101FollowerRobotConfig, SOFollowerConfig

logger = logging.getLogger(__name__)

# SO-101 follower motor layout (see so101.md)
SO101_FOLLOWER_JOINT_TABLE = """
Follower-Arm Axis     Motor  Gear Ratio
--------------------  -----  ----------
Base / Shoulder Pan      1   1 / 345
Shoulder Lift            2   1 / 345
Elbow Flex               3   1 / 345
Wrist Flex               4   1 / 345
Wrist Roll               5   1 / 345
Gripper                  6   1 / 345
"""


# Gripper stall relief. Measured on this gripper holding the cube: commanding the goal 50 ticks past the
# contact point gives ~17% load, >=200 ticks saturates at the 50% torque limit, and a saturated stall held
# ~2 s trips the latched Overload fault. So once the gripper has been blocked while closing for
# GRIPPER_STALL_S, the goal is capped at GRIPPER_HOLD_SQUEEZE_TICKS past where it stopped.
GRIPPER_HOLD_SQUEEZE_TICKS = 50
GRIPPER_STALL_S = 0.5
GRIPPER_STALL_MOVE = 0.5  # moving less than this (0-100 units) between commands counts as blocked

PORT_RECOVERY_STEPS = """\
Next steps: the serial port did not open. Check the USB cable and the COM port
(`lerobot-find-port`), and close any other program using it.
"""

CAMERA_RECOVERY_STEPS = """\
Next steps: a camera failed to open. Check the camera indices (`lerobot-find-cameras opencv`)
and that no other program (or a previous run) is still using them.
"""

MOTOR_RECOVERY_STEPS = """\
Next steps:
  1. Read the error below for the motor ID: 5 = wrist_roll, 6 = gripper.
  2. Power-cycle the arm (unplug motor power ~10 s, replug). A latched Overload/Overheat
     fault only clears when the motor loses power; rerunning will not fix it.
  3. Reseat the 3-pin cable on that motor and the one before it in the daisy chain, and
     check the USB cable and the COM port (`lerobot-find-port`).
  4. If it is the gripper: avoid holding it closed on the object or against its stop, and
     let it cool before retrying.
  5. Stop runs with a single Ctrl+C and wait a few seconds before relaunching.
"""


class SOFollower(Robot):
    """
    Generic SO follower base implementing common functionality for SO-100/101/10X.
    Designed to be subclassed with a per-hardware-model `config_class` and `name`.
    """

    config_class = SO100FollowerRobotConfig
    name = "so_follower"

    def __init__(self, config: SOFollowerConfig):
        super().__init__(config)
        self.config = config
        # choose normalization mode depending on config if available
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        self._gripper_last_pos: float | None = None
        self._gripper_blocked_since: float | None = None

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam in self.cameras:
            if getattr(self.cameras[cam], "use_rgb", True):
                features[cam] = (self.cameras[cam].height, self.cameras[cam].width, 3)
            if getattr(self.cameras[cam], "use_depth", False):
                features[f"{cam}_depth"] = (self.cameras[cam].height, self.cameras[cam].width, 1)
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """

        in_camera_stage = False
        try:
            self.bus.connect()
            if not self.is_calibrated and calibrate:
                logger.info(
                    "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
                )
                self.calibrate()

            in_camera_stage = True
            for cam in self.cameras.values():
                cam.connect()
            in_camera_stage = False

            self.configure()
        except Exception:
            if not self.bus.is_connected:
                steps = PORT_RECOVERY_STEPS
            elif in_camera_stage:
                steps = CAMERA_RECOVERY_STEPS
            else:
                steps = MOTOR_RECOVERY_STEPS
            logger.error(f"{self} failed to connect.\n{steps}")
            self._release_hardware()
            raise
        logger.info(f"{self} connected.")

    def _release_hardware(self) -> None:
        """Best-effort cleanup: torque off + close the port, and release cameras. Never raises."""
        if self.bus.is_connected:
            try:
                self.bus.disconnect(self.config.disable_torque_on_disconnect)
            except Exception as e:
                logger.warning(f"Could not cleanly disconnect the motor bus: {e}")
        for name, cam in self.cameras.items():
            if cam.is_connected:
                try:
                    cam.disconnect()
                except Exception as e:
                    logger.warning(f"Could not disconnect camera '{name}': {e}")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    @property
    def full_turn_motors(self) -> list[str]:
        """Motors that can rotate continuously; use full encoder span instead of recorded range."""
        return ["wrist_roll"]

    def _print_range_of_motion_instructions(self, motors_to_record: list[str]) -> None:
        excluded = [m for m in self.bus.motors if m not in motors_to_record]
        if excluded:
            print(
                f"Move all joints except {', '.join(repr(m) for m in excluded)} sequentially through their "
                "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
            )
        else:
            joint_list = ", ".join(motors_to_record)
            print(
                f"Move all joints ({joint_list}) sequentially through their entire ranges of motion.\n"
                "Recording positions. Press ENTER to stop..."
            )

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        motors_to_record = [motor for motor in self.bus.motors if motor not in self.full_turn_motors]
        self._print_range_of_motion_instructions(motors_to_record)
        range_mins, range_maxes = self.bus.record_ranges_of_motion(motors_to_record)
        for motor in self.full_turn_motors:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, self.config.position_p_coefficient)
                self.bus.write("I_Coefficient", motor, self.config.position_i_coefficient)
                self.bus.write("D_Coefficient", motor, self.config.position_d_coefficient)

                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)  # 50% of max torque to avoid burnout
                    self.bus.write("Protection_Current", motor, 250)  # 50% of max current to avoid burnout
                    self.bus.write("Overload_Torque", motor, 25)  # 25% torque when overloaded
                else:
                    # Arm joints bear load, so keep them stronger than the gripper but still
                    # trip the overload protection (and back off) before stalling at full torque.
                    self.bus.write("Max_Torque_Limit", motor, 1000)  # full holding torque
                    self.bus.write("Protection_Current", motor, 500)  # trip at ~full rated current
                    self.bus.write("Overload_Torque", motor, 40)  # reduce to 40% when overloaded

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                start = time.perf_counter()
                obs_dict[cam_key] = cam.read_latest()
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

            if getattr(cam, "use_depth", False):
                start = time.perf_counter()
                obs_dict[f"{cam_key}_depth"] = cam.read_latest_depth()
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key} depth: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            RobotAction: the action sent to the motors, potentially clipped.
        """

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        if "gripper" in goal_pos:
            goal_pos["gripper"] = self._relieve_gripper_stall(goal_pos["gripper"])

        # Send goal position to the arm
        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def _relieve_gripper_stall(self, goal: float) -> float:
        """Cap how far past an object the gripper is pushed once it is blocked (closing = lower values)."""
        # sync_read ignores the status fault flags, so a latched fault can't crash the control loop here.
        present = self.bus.sync_read("Present_Position", ["gripper"], num_retry=self.config.num_read_retries)[
            "gripper"
        ]
        cal = self.calibration["gripper"]
        squeeze = GRIPPER_HOLD_SQUEEZE_TICKS * 100 / (cal.range_max - cal.range_min)
        moving = self._gripper_last_pos is not None and abs(present - self._gripper_last_pos) > GRIPPER_STALL_MOVE
        self._gripper_last_pos = present

        if moving or goal >= present - squeeze:
            self._gripper_blocked_since = None
            return goal
        now = time.perf_counter()
        if self._gripper_blocked_since is None:
            self._gripper_blocked_since = now
        if now - self._gripper_blocked_since < GRIPPER_STALL_S:
            return goal
        return present - squeeze

    @check_if_not_connected
    def disconnect(self):
        try:
            self.bus.disconnect(self.config.disable_torque_on_disconnect)
        except Exception:
            logger.error(f"{self} failed to disconnect cleanly.\n{MOTOR_RECOVERY_STEPS}")
            raise
        finally:
            for cam in self.cameras.values():
                cam.disconnect()

        logger.info(f"{self} disconnected.")


SO100Follower = SOFollower


class SO101Follower(SOFollower):
    """SO-101 follower: all six joints (including wrist_roll) are range-calibrated."""

    config_class = SO101FollowerRobotConfig

    @property
    def full_turn_motors(self) -> list[str]:
        return []

    def _print_range_of_motion_instructions(self, motors_to_record: list[str]) -> None:
        print("Calibrate each follower joint through its full range of motion:")
        print(SO101_FOLLOWER_JOINT_TABLE)
        print(
            "Move every joint above (including wrist_roll) through its entire range, slowly.\n"
            "Open and close the gripper fully.\n"
            "Recording positions. Press ENTER to stop..."
        )
