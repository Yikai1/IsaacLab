# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import pick_joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.fr5 import FR5_HIGH_PD_CFG  # isort: skip


@configclass
class FR5TVControlPickEnvCfg(pick_joint_pos_env_cfg.FR5TVControlPickEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set FR5 as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FR5_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (FR5)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["j.*"],
            body_name="wrist3_Link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.0]),
        )
