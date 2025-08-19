# from isaacsim import SimulationApp

# simulation_app = SimulationApp({"headless": False})

# import argparse
# import sys
# import cv2
# import numpy as np
# import os
# import matplotlib.pyplot as plt
# import carb
# import numpy as np
# from isaacsim.core.api import World
# from isaacsim.core.utils.prims import create_prim
# from isaacsim.core.api.objects import DynamicCuboid
# from isaacsim.core.utils.stage import add_reference_to_stage
# from isaacsim.robot.manipulators import SingleManipulator
# from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
# from isaacsim.robot.manipulators.grippers import ParallelGripper
# from isaacsim.storage.native import get_assets_root_path
# from isaacsim.sensors.camera import Camera
# import isaacsim.core.utils.numpy.rotations as rot_utils
# from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAAC_LOCAL_DIR

# from isaacsim.core.utils.extensions import enable_extension
# enable_extension("isaacsim.robot_setup.assembler")
# from isaacsim.robot_setup.assembler import RobotAssembler,AssembledRobot

# robot_path = get_assets_root_path()

# my_world = World(stage_units_in_meters=1.0)
# my_world.scene.add_default_ground_plane()
# # env_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
# # add_reference_to_stage(usd_path=env_path, prim_path="/World/Environment")

# robot_path = f"{ISAAC_LOCAL_DIR}/Robots/Fairino/fairino5_v6/fairino5_v6.usd"
# gripper_path = f"{ISAAC_LOCAL_DIR}/Robots/DH/dh_pgc140_nomimic/dh_pgc140_urdf.usd"

# # create_prim(prim_path="/World/Robot", prim_type="Xform", position=[-0.1, 0.45, 0.68])
# add_reference_to_stage(usd_path=robot_path, prim_path ="/World/Robot")

# # create_prim(prim_path="/World/Gripper", prim_type="Xform", position=[0.0, 0.0, 0.0])
# add_reference_to_stage(usd_path=gripper_path, prim_path ="/World/Gripper")

# my_world.reset()
# # robot_assembler = RobotAssembler()
# # assembled_robot = robot_assembler.assemble_articulations(base_robot_path="/World/Robot",
# #                                                        attach_robot_path="/World/Gripper",
# #                                                        base_robot_mount_frame="/wrist3_Link",
# #                                                        attach_robot_mount_frame="/base_link",
# #                                                        fixed_joint_offset = np.array([0.0, 0.0, 0.1]),
# #                                                        fixed_joint_orient = np.array([1.0, 0.0, 0.0, 0.0]),
# #                                                        mask_all_collisions = True,
# #                                                        single_robot=True)

# while simulation_app.is_running():
#     my_world.step(render=True)

# simulation_app.close()

'''
'''

from __future__ import annotations

import numpy as np
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.sim import SimulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaacsim.storage.native import get_assets_root_path
from isaaclab.core.prims import XFormPrim
from isaaclab.core.articulations import Articulation
from isaacsim.robot_setup.assembler import RobotAssembler

@configclass
@configclass
class MyRobotEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 8.3333  # 500 timesteps
    decimation = 2
    num_actions = 6
    num_observations = 23
    num_states = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        disable_contact_processing=True,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=3.0, replicate_physics=True)

    # robot
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path= get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5/ur5.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
               enabled_self_collisions=True, solver_position_iteration_count=12, solver_velocity_iteration_count=1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": -1.712,
                "elbow_joint": 1.712,
                "wrist_1_joint": 0.0,
                "wrist_2_joint": 0.0,
                "wrist_3_joint": 0.0,
            },
            pos=(1.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit= None, 
                effort_limit= None, 
                stiffness= None, 
                damping=  None, 
            ),
        },
    )

class MyRobotEnv(DirectRLEnv):
    # pre-physics step calls
    #   |-- _pre_physics_step(action)
    #   |-- _apply_action()
    # post-physics step calls
    #   |-- _get_dones()
    #   |-- _get_rewards()
    #   |-- _reset_idx(env_ids)
    #   |-- _get_observations()

    cfg: MyRobotEnvCfg

    def __init__(self, cfg: MyRobotEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
    #
    #    
    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self._gripper= Articulation(self.cfg.gripper)
        self.scene.articulations["Robot"] = self._robot

        self.scene.articulations["gripper"] = self._gripper
        print("num envs: ", self.scene.cfg.num_envs)

        for env in range(self.scene.cfg.num_envs):
            robot_assembler = RobotAssembler()
            _ = robot_assembler.assemble_articulations(
                base_robot_path="/World/envs/env_"+str(env)+"/Robot",
                attach_robot_path="/World/envs/env_"+str(env)+"/gripper/gripper_base_link",
                base_robot_mount_frame="/tool0",
                attach_robot_mount_frame="",
                fixed_joint_offset = np.array([-0.04,-0.0, 0.0]),
                fixed_joint_orient  = np.array([1.0, -0.0, -0.0, 1.0]),
                mask_all_collisions = True,
                single_robot = False
            )

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # 设置 headless=True 则不打开 GUI

# 实例化配置和环境
cfg = MyRobotEnvCfg()
env = MyRobotEnv(cfg)

# 重置环境
env.reset()

# 简单执行一定步数动作
for i in range(100):
    actions = env.action_space.sample()  # 随机动作
    env.step(actions)
    if i % 10 == 0:
        print(f"Step {i}")

# 关闭环境和仿真应用
env.close()
simulation_app.close()