# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
# from isaaclab.utils.math import combine_frame_transforms
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def task_done(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    min_x: float = 0.05,
    max_x: float = 0.20,
    min_y: float = -0.21,
    max_y: float = 0.02,
    min_height: float = 0.85,
    min_vel: float = 0.20,
) -> torch.Tensor:
    """Determine if the object placement task is complete.

    This function checks whether all success conditions for the task have been met:
    1. object is within the target x/y range
    2. object is below a minimum height
    3. object velocity is below threshold
    4. Right robot wrist is retracted back towards body (past a given x pos threshold)

    Args:
        env: The RL environment instance.
        object_cfg: Configuration for the object entity.
        right_wrist_max_x: Maximum x position of the right wrist for task completion.
        min_x: Minimum x position of the object for task completion.
        max_x: Maximum x position of the object for task completion.
        min_y: Minimum y position of the object for task completion.
        max_y: Maximum y position of the object for task completion.
        min_height: Minimum height (z position) of the object for task completion.
        min_vel: Minimum velocity magnitude of the object for task completion.

    Returns:
        Boolean tensor indicating which environments have completed the task.
    """
    # Get object entity from the scene
    object: RigidObject = env.scene[object_cfg.name]

    # Extract wheel position relative to environment origin
    wheel_x = object.data.root_pos_w[:, 0] - env.scene.env_origins[:, 0]
    wheel_y = object.data.root_pos_w[:, 1] - env.scene.env_origins[:, 1]
    wheel_height = object.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    wheel_vel = torch.abs(object.data.root_vel_w)

    # Check all success conditions and combine with logical AND
    done = wheel_x < max_x
    done = torch.logical_and(done, wheel_x > min_x)
    done = torch.logical_and(done, wheel_y < max_y)
    done = torch.logical_and(done, wheel_y > min_y)
    done = torch.logical_and(done, wheel_height < min_height)
    done = torch.logical_and(done, wheel_vel[:, 0] < min_vel)
    done = torch.logical_and(done, wheel_vel[:, 1] < min_vel)
    done = torch.logical_and(done, wheel_vel[:, 2] < min_vel)

    return done


def root_height_above_minimum(
    env: ManagerBasedRLEnv,
    minimum_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("tvcontrol")
) -> torch.Tensor:
    """Terminate when the asset's root height is above the minimum height.

    Note:
        This is currently only supported for flat terrains, i.e. the minimum height is in the world frame.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    return asset.data.root_pos_w[:, 2] > minimum_height
