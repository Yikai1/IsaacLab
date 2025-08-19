# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym
import os

from . import (
    agents,
    pick_ik_rel_env_cfg,
    pick_joint_pos_env_cfg,
    pick_ik_rel_visuomotor_env_cfg,

)

gym.register(
    id="Isaac-Pick-Cube-FR5-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pick_joint_pos_env_cfg.FR5CubePickEnvCfg,
    },
    disable_env_checker=True,
)
gym.register(
    id="Isaac-Pick-Cube-FR5-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pick_ik_rel_env_cfg.FR5CubePickEnvCfg,
        "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc_rnn_low_dim.json"),
    },
    disable_env_checker=True,
)
gym.register(
    id="Isaac-Pick-Cube-FR5-IK-Rel-Visuomotor-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pick_ik_rel_visuomotor_env_cfg.FR5CubePickVisuomotorEnvCfg,
        "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc_rnn_low_dim.json"),
    },
    disable_env_checker=True,
)