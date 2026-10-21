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
"""Normalizes euler->quaternion conversion across Isaac Sim versions.

`isaacsim.core.utils.rotations` (Isaac Sim 5.x, and 6.0.1 under a normal Kit
launch) provides `euler_angles_to_quat`. Isaac Lab 3.0's kit-less launch path
doesn't register that extension, so the module fails to import there; the
Core Experimental equivalent (`euler_angles_to_quaternion`) is used instead.
Both return (w, x, y, z); the experimental one returns a `wp.array`.
"""
import numpy as np


def euler_angles_to_quat(euler_angles: np.ndarray, degrees: bool = False) -> np.ndarray:
    try:
        from isaacsim.core.utils.rotations import euler_angles_to_quat as _euler_angles_to_quat

        return _euler_angles_to_quat(euler_angles, degrees=degrees)
    except ImportError:
        from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion

        return euler_angles_to_quaternion(euler_angles, degrees=degrees).numpy()
