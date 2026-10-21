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
"""Detects a sustained real-world gripper hold from commanded-vs-measured
gripper position, deliberately isaac-free so it stays unit-testable (matches
utils/geometry.py's convention) -- see the real->sim mirroring plan's Step 3
grasp-latch fallback.

Derived empirically from a real recorded episode (eval_sanity-act,
so101-lerobot, ACT policy): a successful hold pins `measured` at a constant
plateau (the grasped object's width, in gripper units) while `commanded`
keeps driving toward fully closed; a failed grasp attempt closes both all
the way to near-zero instead. The default thresholds below came from that
one episode's ~29-unit plateau -- retune `measured_threshold` to roughly
`plateau - 5` if a different cube/gripper combination's plateau value reads
noticeably different.
"""


class GraspDetector:
    """Stateful per-tick detector. Call `update()` once per frame with that
    frame's commanded/measured gripper values (same units for both, e.g.
    raw gripper.pos)."""

    def __init__(
        self,
        gap_threshold: float = 15.0,
        measured_threshold: float = 15.0,
        delta_threshold: float = 0.5,
        hold_frames: int = 10,
    ):
        self.gap_threshold = gap_threshold
        self.measured_threshold = measured_threshold
        self.delta_threshold = delta_threshold
        self.hold_frames = hold_frames
        self._prev_measured: float | None = None
        self._plateau_count = 0
        self.holding = False

    def update(self, commanded: float, measured: float) -> bool:
        """Returns the updated `holding` state (also available as
        `self.holding` afterward)."""
        gap = measured - commanded
        delta = 0.0 if self._prev_measured is None else abs(measured - self._prev_measured)
        self._prev_measured = measured

        plateaued = (
            gap > self.gap_threshold
            and measured > self.measured_threshold
            and delta < self.delta_threshold
        )
        self._plateau_count = self._plateau_count + 1 if plateaued else 0
        self.holding = self._plateau_count >= self.hold_frames
        return self.holding
