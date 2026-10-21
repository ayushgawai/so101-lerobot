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
"""Pure-torch geometry helpers, deliberately free of any isaaclab/isaacsim
import so they stay unit-testable without a booted Kit process.

Quaternions are (w, x, y, z), matching isaaclab.utils.math's convention.
"""
import torch


def quat_conjugate(quat: torch.Tensor) -> torch.Tensor:
    """Conjugate of a unit quaternion batch, shape (..., 4), wxyz."""
    w, x, y, z = quat.unbind(-1)
    return torch.stack([w, -x, -y, -z], dim=-1)


def quat_apply(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate a batch of vectors (..., 3) by a batch of unit quaternions (..., 4), wxyz."""
    qw = quat[..., 0:1]
    qvec = quat[..., 1:4]
    t = 2.0 * torch.cross(qvec, vec, dim=-1)
    return vec + qw * t + torch.cross(qvec, t, dim=-1)


def point_in_local_box(
    point_pos_w: torch.Tensor,
    ref_pos_w: torch.Tensor,
    ref_quat_w: torch.Tensor,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_max: float,
    z_min: float = float("-inf"),
) -> torch.Tensor:
    """Whether each world-frame point falls inside an axis-aligned box defined
    in a per-env reference frame's local coordinates.

    Args:
        point_pos_w: World-frame point positions, shape (num_envs, 3).
        ref_pos_w: World-frame position of the reference frame's origin, shape (num_envs, 3).
        ref_quat_w: World-frame orientation of the reference frame, shape (num_envs, 4), wxyz.
        x_min/x_max/y_min/y_max: Local-frame XY bounds of the box.
        z_max: Local-frame Z upper bound (exclusive), e.g. a container's rim/slot-entry height.
        z_min: Local-frame Z lower bound (inclusive). Defaults to unbounded below.

    Returns:
        Boolean tensor of shape (num_envs,).
    """
    local_pos = quat_apply(quat_conjugate(ref_quat_w), point_pos_w - ref_pos_w)
    x, y, z = local_pos[..., 0], local_pos[..., 1], local_pos[..., 2]
    return (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max) & (z < z_max) & (z >= z_min)
