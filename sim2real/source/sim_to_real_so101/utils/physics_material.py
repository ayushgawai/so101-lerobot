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
"""Apply friction/restitution to a collision-shape prim, for the raw-Isaac-Sim
scripts' runtime grasp-friction patches (AWSBuilderCube, gripper/jaw pads).

Requires a booted Kit process (imports pxr) -- unlike utils/geometry.py,
which is deliberately kept isaacsim-free.
"""
from pxr import Usd, UsdPhysics


def apply_friction_material(
    prim: Usd.Prim, static_friction: float, dynamic_friction: float, restitution: float = 0.0
) -> None:
    """Apply/update a PhysicsMaterialAPI on a collision-shape prim.

    Must target the prim with PhysicsCollisionAPI (a mesh, or a collision
    Xform like SO-ARM101's gripper/jaw pads), not a RigidBodyAPI root --
    friction/restitution are per-shape properties in USD Physics, distinct
    from the body-level RigidBodyAPI/MassAPI.
    """
    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(prim)
    material = UsdPhysics.MaterialAPI(prim)
    material.CreateStaticFrictionAttr().Set(static_friction)
    material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    material.CreateRestitutionAttr().Set(restitution)
