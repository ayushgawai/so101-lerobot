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
"""AWSBuilderCube -> PaperBowl teleop task
(docs/aws-cube-to-bowl-teleop-plan.md §4), staged like vials_to_rack_env_cfg.py:

- Stage 1 (AwsCubeToBowlSceneCfg / AwsCubeToBowlEnvCfg): bare teleop, no
  sensors/cameras/observations beyond the base ObservationsCfg -- validates
  the real unknown (does the cube's rigid-body physics behave sensibly under
  gripper contact) before adding any bookkeeping on top.
- Stage 2 (AwsCubeToBowlDatasetSceneCfg / AwsCubeToBowlDatasetEnvCfg):
  contact sensor + grasp/placement detection + pose randomization on reset,
  for dataset-recording runs.
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from sim_to_real_so101 import assets
from sim_to_real_so101.assets.so101 import S0101_CONTACT_GRASP_CFG
from sim_to_real_so101.mdp import cube_grasped, cube_placed_in_bowl, reset_aws_cube

from .so101_env_cfg import EventCfg, LerobotSo101BaseSceneCfg, ObservationsCfg, SO101TeleopEnvCfg

assets_path = os.path.dirname(os.path.abspath(assets.__file__))

# Exact xformOp:translate / xformOp:orient authored on /World/SO_ARM101_USD
# in real-to-sim.usd (verified directly against the raw prim, not carried
# over from SO101_CFG's own init_state -- that rot=90deg-yaw value is tuned
# for a *different* task's scene layout and does not apply here; the two
# fields were previously mixed -- real position + wrong-task rotation --
# which visually mounted the robot incorrectly).
ROBOT_POS = (0.0, 0.3, 0.72)
ROBOT_ROT = (1.0, 0.0, 0.0, 0.0)  # identity, wxyz

# Matches /World/AWSBuilderCube's world position in real-to-sim.usd (the
# cube's own center, not /World/AWSCubePaper's position -- that sibling
# decal sits at the cube's bottom face, z=0.7504, 2.5cm lower), i.e. where
# the cube rests on the table before being picked up.
AWS_CUBE_POS = (0.0, 0.03, 0.7754)


@configclass
class AwsCubeToBowlSceneCfg(LerobotSo101BaseSceneCfg):
    """Scene built off the base robot-only scene, not SO101TaskSceneCfg --
    that one assumes the indoor-room/mat/lightstudio layout, a different
    physical setup than this two-table/RobotMount demo.
    """

    # Contact sensors enabled at spawn even though no ContactSensorCfg is
    # added until stage 2, so the sensor added later doesn't silently read
    # zero force because activate_contact_sensors was never set.
    robot: ArticulationCfg = S0101_CONTACT_GRASP_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=S0101_CONTACT_GRASP_CFG.init_state.replace(pos=ROBOT_POS, rot=ROBOT_ROT),
    )

    # Thin reference wrapper onto real-to-sim.usd's room/tables/mount/bowl,
    # with the embedded robot and cube deactivated (see
    # docs/aws-cube-to-bowl-teleop-plan.md §3) -- PaperBowl stays static and
    # untouched, no separate bowl asset needed.
    room = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_path}/usd/aws-cube-bowl-room.usda",
        ),
    )

    # PaperBowl (nested under "room" above) is read directly off its raw
    # prim in cube_placed_in_bowl (mdp/terms.py) rather than registered here
    # as its own scene entity -- see that function's docstring for why.

    # Separate dynamic rigid-body copy of the cube (the embedded one in the
    # room wrapper above is deactivated) so PhysX actually simulates the
    # grasp via contact/friction forces instead of the gripper colliding
    # with static geometry like a wall. Named "AWSBuilderCubeDynamic", not
    # "AWSBuilderCube", to avoid colliding with the room wrapper's
    # deactivated same-named prim at .../Room/AWSBuilderCube -- confirmed
    # PhysX's rigid-contact-view filter glob isn't segment-strict (its `*`
    # crosses `/` boundaries), so a same-leaf-name inactive prim elsewhere in
    # the tree makes filter_prim_paths_expr resolve "found 2" instead of 1
    # and silently fail contact-sensor init (omni.physx.tensors.plugin log).
    aws_cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/AWSBuilderCubeDynamic",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_path}/usd/AWSBuilderCube.usda",
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=AWS_CUBE_POS),
    )


@configclass
class AwsCubeToBowlEnvCfg(SO101TeleopEnvCfg):
    """Stage 1: bare teleop -- inherits actions/observations/events/sim
    settings from SO101TeleopEnvCfg unchanged, overriding the scene and the
    default viewer.
    """

    scene: AwsCubeToBowlSceneCfg = AwsCubeToBowlSceneCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        # SO101TeleopEnvCfg's default viewer (eye=(-0.25,-0.4,0.22),
        # lookat=(0.15,0,0.12)) is tuned for the original compact tabletop
        # task, where the robot sits near the ground. This task's robot sits
        # at ROBOT_POS z=0.72 (a real two-table RobotMount setup) -- left
        # unchanged, the default viewer looks up at the scene from below
        # table height. Placed above and behind the robot instead, looking
        # down at the robot/cube/bowl working area (cube z=0.7754, bowl at
        # roughly (0.2, 0.03, 0.75) in real-to-sim.usd).
        self.viewer.eye = (0.5, -0.3, 1.15)
        self.viewer.lookat = (0.05, 0.15, 0.75)


@configclass
class AwsCubeToBowlDatasetSceneCfg(AwsCubeToBowlSceneCfg):
    """Stage 2 scene: adds the contact sensor needed for grasp/placement
    detection, only relevant once dataset recording is wanted."""

    contact_grasp = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/jaw",
        update_period=0.0,
        history_length=1,
        debug_vis=False,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/AWSBuilderCubeDynamic"],
    )


@configclass
class AwsCubeToBowlDatasetEventCfg(EventCfg):
    """Configuration for events."""

    reset_aws_cube_pose = EventTerm(
        func=reset_aws_cube,
        mode="reset",
        params={
            "aws_cube": "aws_cube",
            "pose_range": {
                "x": (-0.03, 0.03),
                "y": (-0.03, 0.03),
                "yaw": (-0.3, 0.3),
            },
        },
    )


@configclass
class AwsCubeToBowlDatasetObservationsCfg(ObservationsCfg):
    """Configuration for observations."""

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask tracking."""

        cube_grasped_obs = ObsTerm(
            func=cube_grasped,
            params={
                "contact_sensor_cfg": SceneEntityCfg("contact_grasp"),
                "aws_cube": "aws_cube",
                # AWS_CUBE_POS.z = 0.7754 (resting height) -- unverified
                # estimate, needs real-teleop tuning (mirrors vial task's own
                # "check debug output for actual resting height" note).
                "min_height": 0.79,
                "warmup_steps": 30,
                "force_threshold": 2,  # N
            },
        )

        cube_placed = ObsTerm(
            func=cube_placed_in_bowl,
            params={
                "contact_sensor_cfg": SceneEntityCfg("contact_grasp"),
                "aws_cube": "aws_cube",
                "bowl_prim_path": "{ENV_REGEX_NS}/Room/PaperBowl",
                "warmup_steps": 30,
                "grasp_history_window": 20,
                "force_threshold": 2,  # N
                # PaperBowl local extent (docs/aws-cube-to-bowl-teleop-plan.md §2)
                "bowl_local_x_min": -0.05,
                "bowl_local_x_max": 0.05,
                "bowl_local_y_min": -0.0375,
                "bowl_local_y_max": 0.0375,
                "bowl_local_z_max": 0.032,
            },
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = False

    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class AwsCubeToBowlDatasetEnvCfg(AwsCubeToBowlEnvCfg):
    """Stage 2: dataset-recording support."""

    scene: AwsCubeToBowlDatasetSceneCfg = AwsCubeToBowlDatasetSceneCfg()
    events: AwsCubeToBowlDatasetEventCfg = AwsCubeToBowlDatasetEventCfg()
    observations: AwsCubeToBowlDatasetObservationsCfg = AwsCubeToBowlDatasetObservationsCfg()
