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
"""Normalizes Isaac Lab asset/sensor `.data.*` outputs to torch.Tensor.

Isaac Lab 2.x returns torch.Tensor from `.data.*` properties. Isaac Lab 3.0
returns `isaaclab.utils.warp.proxy_array.ProxyArray` (a dual torch/warp
wrapper around a raw `warp.array`, exposing an explicit `.torch` accessor)
-- confirmed directly against a live Isaac Lab 3.0/Isaac Sim 6.0.1 process,
not just `warp.array` itself as an earlier reading of the migration notes
assumed. This keeps the mdp package working unmodified on any of the three.

IMPORTANT -- quaternion convention: Isaac Lab 3.0 also changed `.data.*`
quaternions from (w, x, y, z) to (x, y, z, w) (see ProxyArray's own
WARN_ON_TORCH_QUATF_ACCESS docstring). as_torch() only normalizes the
*container type*; it does NOT reorder quaternion components. Every
quaternion produced or consumed by this package (sim_to_real_so101.utils.
geometry, rotations_compat, mdp/terms.py's vertical-orientation checks,
so101.py's InitialStateCfg.rot, etc.) still assumes (w, x, y, z) throughout,
matching Isaac Lab 2.x and this repo's originally-targeted isaacsim.core
utilities. This has NOT been audited for correctness under Isaac Lab 3.0 --
treat any quaternion math in this repo as unverified, not silently correct,
when running under Isaac Lab 3.0/Isaac Sim 6.0.1 until that audit happens.
"""
import torch


def as_torch(x):
    if isinstance(x, torch.Tensor):
        return x
    try:
        import warp as wp

        if isinstance(x, wp.array):
            return wp.to_torch(x)
    except ImportError:
        pass
    if hasattr(x, "torch") and hasattr(x, "warp"):
        # isaaclab.utils.warp.proxy_array.ProxyArray (Isaac Lab 3.0's dual
        # torch/warp accessor) duck-typed rather than imported directly, to
        # avoid a hard isaaclab-version-specific import in this module.
        return x.torch
    return x
