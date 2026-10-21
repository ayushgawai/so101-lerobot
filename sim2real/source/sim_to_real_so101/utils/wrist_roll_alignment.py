# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Wrist-roll leader-to-sim zero-pose alignment -- pure math, no isaacsim/
omni/lerobot/torch import (same "isaac-free, unit-testable" convention as
utils/fixed_workspace.py and utils/geometry.py).

Root cause (2026-08-24, see leader_arm_teleop_raw_isaacsim.py's wrist_roll
debug printout): LeRobotSO101Interface.get_mapped_actions_vectorized()
(lerobot_interface.py) maps every joint through
``mapped_deg = joint_min + ((raw + 100) / 200) * (joint_max - joint_min)``,
which for a symmetric USD range (wrist_roll: -160..160) passes exactly
through the origin -- raw=0 always maps to mapped_deg=0, with no per-joint
zero offset. That is a pure scale, confirmed exactly against the observed
logs (mapped_deg == 1.6 * raw for every sample). The scale itself is correct
and intentional (it is what makes the leader's -100..100 reading cover the
USD asset's full authored -160..160 deg travel) -- it is not the bug.

The bug is that this scale-through-zero assumption silently requires
raw=0 (the leader's calibration zero) to coincide with the simulated
Wrist_Roll joint's own authored 0 deg pose. That holds for the other five
joints, but not for Wrist_Roll: real-to-sim.usd's Wrist_Roll joint's 0 deg
pose does not visually correspond to the real wrist's calibrated-neutral
orientation -- confirmed by direct observation (physical wrist upright vs.
simulated wrist rotated ~90 deg at near-zero leader/sim readings), and
consistent with assets/so101.py's Isaac-Lab ArticulationCfg, whose
init_state park Wrist_Roll at -1.6034 rad (~-91.9 deg) rather than 0 --
i.e. nothing else in this codebase treats this joint's USD zero as the
robot's visual "neutral" either. wrist_roll is also lerobot's sole
``full_turn_motor`` (hardcoded calibration range_min=0/range_max=4095,
a full revolution, instead of the other five joints' motion-capture-derived
range) so unlike them, its raw=0 is defined purely by wherever the arm
happened to be held during calibration's homing step, with no mechanical
range-of-motion anchor -- another reason its zero can drift out of sync with
a USD asset's authored rest pose independently of the other joints.

Fix: an explicit, configurable direction/offset correction applied AFTER
SO101_USD_MAPPING's scale (i.e. on top of the already-scaled degree value
from get_mapped_actions_vectorized) and BEFORE joint-limit clamping::

    unclamped_deg = direction_sign * scaled_deg + zero_offset_deg
    clamped_deg = clamp(unclamped_deg, lower_limit_deg, upper_limit_deg)

Deliberately NOT applied inside get_mapped_actions_vectorized() itself:
that method is shared by every script in this repo (replay_act_dataset_to_
sim.py, test_grasp_dynamics.py, compare_real_vs_sim_joints.py, the GR00T
client, ...); this module keeps the correction scoped to the one script
that has this issue (leader_arm_teleop_raw_isaacsim.py), per the same
"only wrist_roll changes, nothing else does" requirement that keeps this
fix minimal.

Defaults (direction_sign=+1.0, zero_offset_deg=0.0) reproduce today's
unmodified passthrough -- deliberately NOT hardcoded to +/-90 deg, because
the correct value cannot be determined from the USD file or the calibration
JSON alone (a single revolute axis makes +90 and -90 equally plausible
without a live reference). Determine it on real hardware:

  1. Run leader_arm_teleop_raw_isaacsim.py and hold the physical wrist in
     its upright/neutral pose -- watch the periodic "wrist_roll --" debug
     line for `leader raw` near 0.
  2. Compare the simulated wrist's orientation to the real one at that
     moment. If the sim is rotated ~90 deg one way, set
     WRIST_ROLL_ZERO_OFFSET_DEG (in that script) to +90.0; if rotated the
     other way, to -90.0. Re-run and confirm the sim now looks upright at
     leader raw~0.
  3. Rotate the physical wrist a known amount (e.g. slowly toward one
     mechanical limit) and confirm the simulated wrist turns the *same*
     direction by roughly the *same* amount (accounting for the 1.6x
     scale). If it turns the opposite direction instead, the zero pose is
     right but the axis is mirrored -- set WRIST_ROLL_DIRECTION_SIGN to
     -1.0 instead of negating the offset.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class WristRollTarget:
    """Every intermediate value of one wrist_roll alignment application --
    exactly what leader_arm_teleop_raw_isaacsim.py's debug printout reports
    (raw leader angle is logged separately by the caller, which already has
    it from `real_action["wrist_roll.pos"]`)."""

    scaled_deg: float
    direction_sign: float
    zero_offset_deg: float
    unclamped_deg: float
    clamped_deg: float


@dataclass(frozen=True)
class WristRollAlignment:
    """Configurable direction/zero-offset correction. See module docstring
    for the root cause and how to determine the right values on hardware.

    direction_sign: +1.0 (default) or -1.0. Flip this if, after
        zero_offset_deg already makes the *neutral* pose match, the
        simulated wrist rotates the *opposite* way from the real one as you
        move it away from neutral.
    zero_offset_deg: degrees added after direction_sign is applied to the
        already-scaled (SO101_USD_MAPPING) degree value. Default 0.0
        (today's unmodified passthrough) -- see module docstring for how to
        find the right value on real hardware.
    """

    direction_sign: float = 1.0
    zero_offset_deg: float = 0.0

    def apply(self, scaled_deg: float, lower_limit_deg: float, upper_limit_deg: float) -> WristRollTarget:
        unclamped_deg = self.direction_sign * scaled_deg + self.zero_offset_deg
        clamped_deg = max(lower_limit_deg, min(upper_limit_deg, unclamped_deg))
        return WristRollTarget(
            scaled_deg=scaled_deg,
            direction_sign=self.direction_sign,
            zero_offset_deg=self.zero_offset_deg,
            unclamped_deg=unclamped_deg,
            clamped_deg=clamped_deg,
        )
