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
"""Prints an unmissable console banner naming the actual Isaac Sim version in
use. Isaac Lab has no simulator of its own -- it's a framework that drives
Isaac Sim -- so both version strings are shown, labeled by role, to make
clear which one is the simulator actually rendering/simulating. When run
against a plain Kit-app install (e.g. C:\\Isaac-Sim, not pip-installed),
`isaacsim` has no importlib.metadata entry, so a VERSION-file fallback is
used instead -- see _find_isaacsim_version().
"""
import importlib.metadata
import os


def _find_isaacsim_version() -> str:
    try:
        return importlib.metadata.version("isaacsim")
    except importlib.metadata.PackageNotFoundError:
        pass

    # Kit-app installs (e.g. C:\Isaac-Sim, launched via its own python.bat)
    # aren't pip packages -- python.bat sets ISAAC_PATH to the install root,
    # which has a plain-text VERSION file directly in it.
    isaac_path = os.environ.get("ISAAC_PATH")
    if isaac_path:
        version_file = os.path.join(isaac_path, "VERSION")
        if os.path.isfile(version_file):
            with open(version_file, encoding="utf-8") as f:
                return f.read().strip()

    # Fall back to walking up from the isaacsim package's own directory,
    # in case ISAAC_PATH wasn't set by whatever launched this process.
    try:
        import isaacsim

        directory = os.path.dirname(os.path.abspath(isaacsim.__file__))
        for _ in range(6):
            version_file = os.path.join(directory, "VERSION")
            if os.path.isfile(version_file):
                with open(version_file, encoding="utf-8") as f:
                    return f.read().strip()
            parent = os.path.dirname(directory)
            if parent == directory:
                break
            directory = parent
    except ImportError:
        pass

    return "UNKNOWN"


def print_simulator_version_banner() -> None:
    isaacsim_version = _find_isaacsim_version()
    try:
        isaaclab_version = importlib.metadata.version("isaaclab")
        framework_line = (
            f"  FRAMEWORK : Isaac Lab {isaaclab_version}  (drives Isaac Sim -- not a separate simulator)"
        )
    except importlib.metadata.PackageNotFoundError:
        framework_line = "  FRAMEWORK : none (raw Isaac Sim -- no Isaac Lab installed in this environment)"

    width = 60
    print("=" * width)
    print(f"  SIMULATOR : Isaac Sim {isaacsim_version}")
    print(framework_line)
    print("=" * width)
