# Copyright (c) 2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from scipy.spatial.transform import Rotation as R
import isaaclab.sim as sim_utils
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
# from isaaclab.utils.math import quat_from_euler_xyz
from ... import mdp
from . import pick_joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.fr5 import FR5_HIGH_PD_CFG  # isort: skip


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object = ObsTerm(func=mdp.object_obs)
        tvcontrol_positions = ObsTerm(func=mdp.cube_positions_in_world_frame)
        tvcontrol_orientations = ObsTerm(func=mdp.cube_orientations_in_world_frame)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)
        table_cam = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("table_cam"), "data_type": "rgb", "normalize": False}
        )
        wrist_cam = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("wrist_cam"), "data_type": "rgb", "normalize": False}
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""

        grasp = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("tvcontrol"),
            },
        )
        # grasp = ObsTerm(
        #     func=mdp.object_grasped_height,
        #     params={
        #         "robot_cfg": SceneEntityCfg("robot"),
        #         "ee_frame_cfg": SceneEntityCfg("ee_frame"),
        #         "object_cfg": SceneEntityCfg("tvcontrol"),
        #         "minimum_height": 1.1,
        #     },
        # )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class FR5TVControlPickVisuomotorEnvCfg(pick_joint_pos_env_cfg.FR5TVControlPickEnvCfg):
    observations: ObservationsCfg = ObservationsCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FR5_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["j.*"],
            body_name="wrist3_Link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.0]),
        )

        # Set cameras
        # Set wrist camera
        self.scene.wrist_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wrist3_Link/wrist_cam",
            update_period=0.0,
            height=360,
            width=640,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18.14756, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.01, 1000000.0)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(-0.08, 0, 0.13), rot=(-0.68301, 0.18301, -0.18301, 0.68301), convention="ros"
            ),
        )

        # Set table view camera
        # self.scene.table_cam = CameraCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/table_cam",
        #     update_period=0.0,
        #     height=360,
        #     width=640,
        #     data_types=["rgb", "distance_to_image_plane"],
        #     spawn=sim_utils.PinholeCameraCfg(
        #         focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 2)
        #     ),
        #     offset=CameraCfg.OffsetCfg(
        #         pos=(-1.5, -0.014, 1.0), rot=(0.2706, -0.65328, 0.65328, -0.2706), convention="ros"
        #     ),
        # )

        self.scene.table_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/table_cam",
            update_period=0.0,
            height=360,
            width=640,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18.14756, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.01, 1000000.0)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(-1.5, 0.5, 1.2), rot=(0.26489, -0.65562, 0.65562, -0.26489), convention="ros"
            ),
        )

        # Set settings for camera rendering
        self.rerender_on_reset = True
        self.sim.render.antialiasing_mode = "OFF"  # disable dlss

        # List of image observations in policy observations
        self.image_obs_list = ["table_cam", "wrist_cam"]
