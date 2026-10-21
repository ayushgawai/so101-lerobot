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
"""Unit tests for utils/wrist_roll_alignment.py -- pure math, no isaacsim/
omni/lerobot/torch needed, so this runs under any plain Python with pytest
(e.g. the repo's usdenv), not just C:\\Isaac-Sim\\python.bat:

    pytest source/sim_to_real_so101/utils/test_wrist_roll_alignment.py

wrist_roll's real USD joint limits are -160..160 deg
(LeRobotSO101Interface.SO101_USD_MAPPING["wrist_roll"] in
lerobot_interface.py) -- reproduced here as a literal constant rather than
imported, so this test file stays import-light (importing lerobot_interface
pulls in torch and the vendored lerobot fork).
"""
from sim_to_real_so101.utils.wrist_roll_alignment import WristRollAlignment

WRIST_ROLL_LOWER_DEG = -160.0
WRIST_ROLL_UPPER_DEG = 160.0


def test_leader_neutral_maps_to_simulator_neutral_with_default_alignment():
    # Default alignment (direction=+1, offset=0) is today's unmodified
    # passthrough: leader raw=0 -> scaled_deg=0 -> sim target 0 deg.
    alignment = WristRollAlignment()
    target = alignment.apply(0.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert target.unclamped_deg == 0.0
    assert target.clamped_deg == 0.0


def test_leader_neutral_maps_to_configured_sim_neutral_with_an_offset():
    # A non-zero zero_offset_deg shifts where leader raw=0 lands in sim --
    # this is the knob the hardware verification procedure (module
    # docstring) tunes.
    alignment = WristRollAlignment(direction_sign=1.0, zero_offset_deg=90.0)
    target = alignment.apply(0.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert target.unclamped_deg == 90.0
    assert target.clamped_deg == 90.0


def test_positive_and_negative_rotations_map_in_the_expected_direction():
    alignment = WristRollAlignment()
    positive = alignment.apply(50.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    negative = alignment.apply(-50.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert positive.unclamped_deg > 0
    assert negative.unclamped_deg < 0
    assert positive.unclamped_deg == -negative.unclamped_deg


def test_direction_sign_flips_the_mapped_rotation():
    positive_dir = WristRollAlignment(direction_sign=1.0)
    negative_dir = WristRollAlignment(direction_sign=-1.0)
    scaled_deg = 50.0
    target_pos = positive_dir.apply(scaled_deg, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    target_neg = negative_dir.apply(scaled_deg, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert target_pos.unclamped_deg == 50.0
    assert target_neg.unclamped_deg == -50.0


def test_targets_are_clamped_within_the_simulators_joint_limits():
    alignment = WristRollAlignment(direction_sign=1.0, zero_offset_deg=0.0)

    # Within range: unclamped == clamped.
    within = alignment.apply(100.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert within.unclamped_deg == 100.0
    assert within.clamped_deg == 100.0

    # Past the upper limit: clamp to it, but unclamped_deg still reports the
    # true (pre-clamp) value for the debug printout.
    over_upper = alignment.apply(250.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert over_upper.unclamped_deg == 250.0
    assert over_upper.clamped_deg == WRIST_ROLL_UPPER_DEG

    # Past the lower limit, symmetric case.
    under_lower = alignment.apply(-250.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert under_lower.unclamped_deg == -250.0
    assert under_lower.clamped_deg == WRIST_ROLL_LOWER_DEG


def test_zero_offset_can_push_a_target_past_the_limit_and_it_still_clamps():
    # A configured zero_offset_deg near a joint limit plus a large leader
    # rotation must still clamp -- offset and direction are applied before
    # clamping, never after, per the module's documented order of
    # operations.
    alignment = WristRollAlignment(direction_sign=1.0, zero_offset_deg=90.0)
    target = alignment.apply(100.0, WRIST_ROLL_LOWER_DEG, WRIST_ROLL_UPPER_DEG)
    assert target.unclamped_deg == 190.0
    assert target.clamped_deg == WRIST_ROLL_UPPER_DEG
