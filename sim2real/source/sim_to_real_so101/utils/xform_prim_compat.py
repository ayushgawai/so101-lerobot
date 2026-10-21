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
"""Normalizes construction of a path-pattern-driven Xform prim view across
Isaac Sim versions.

`isaacsim.core.prims.XFormPrim` (Isaac Sim 5.x, and 6.0.1 under a normal Kit
launch) takes `prim_paths_expr`. Isaac Lab 3.0's kit-less launch path doesn't
enable the extension providing it by default, so the module fails to import
there (confirmed directly); the Core Experimental equivalent
(`isaacsim.core.experimental.prims.XformPrim`, kwarg `paths`) is used
instead, with its extension enabled on demand -- confirmed that a plain
import of it also fails until its owning extension is explicitly enabled via
the Kit extension manager, unlike the always-loaded old module.
"""


def make_xform_prim(prim_paths_expr: str):
    try:
        from isaacsim.core.prims import XFormPrim

        return XFormPrim(prim_paths_expr=prim_paths_expr)
    except ImportError:
        import omni.kit.app

        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
            "isaacsim.core.experimental.prims", True
        )
        from isaacsim.core.experimental.prims import XformPrim

        return XformPrim(paths=prim_paths_expr)
