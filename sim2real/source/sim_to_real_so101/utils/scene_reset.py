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
"""Cache/restore a prim's authored pose, for a "reset scene props to their
original position" keyboard action in the raw-Isaac-Sim scripts.

Requires a booted Kit process (imports pxr) -- unlike utils/geometry.py,
which is deliberately kept isaacsim-free.
"""
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def snapshot_xform_ops(prim: Usd.Prim) -> list[tuple]:
    """Capture a prim's current xformOps as (op, value) pairs, in authored
    order -- generic over whichever ops are actually authored (translate-only,
    translate+orient, translate+rotateXYZ+scale, ...) rather than assuming a
    fixed set of op names."""
    return [(op, op.Get()) for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]


def ensure_translate_orient_ops(prim: Usd.Prim) -> tuple[Usd.Attribute, Usd.Attribute]:
    """Return (translate_attr, orient_attr) for `prim`, adding a
    xformOp:orient if one isn't already authored -- without touching
    whatever ops already exist or duplicating one that does.

    Some prims in real-to-sim.usd only author xformOp:translate (e.g.
    AWSBuilderCube, confirmed by direct inspection); others already have
    translate+orient+scale (e.g. PaperBowl). Adding an orient op when
    missing appends it after the existing ops in xformOpOrder -- since
    translate is always first here, the result is [translate, orient, ...],
    i.e. the composed local transform is T * R (rotate the prim about its
    own origin, then translate to its world position), matching the
    convention PaperBowl already uses.
    """
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    translate_op = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    if translate_op is None:
        raise RuntimeError(f"{prim.GetPath()} has no xformOp:translate -- cannot safely apply a pose to it")
    orient_op = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
    if orient_op is None:
        orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)
        orient_op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return translate_op.GetAttr(), orient_op.GetAttr()


def restore_prim_pose(prim: Usd.Prim, snapshot: list[tuple], zero_velocity: bool = False) -> None:
    """Restore a prim's xformOps from a snapshot taken by snapshot_xform_ops.

    Args:
        zero_velocity: Also zero physics:velocity/physics:angularVelocity --
            needed for a dynamic RigidBodyAPI prim (e.g. AWSBuilderCube) so it
            doesn't carry momentum from before the reset into the next
            physics step. Leave False for a static prim (e.g. PaperBowl),
            which never had RigidBodyAPI applied.
    """
    for op, value in snapshot:
        op.Set(value)
    if zero_velocity:
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        rigid_body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rigid_body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
