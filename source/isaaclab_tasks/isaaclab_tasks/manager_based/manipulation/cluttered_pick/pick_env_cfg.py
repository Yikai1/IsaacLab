# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_LOCAL_DIR
from isaaclab.assets import RigidObjectCfg
from . import mdp

##
# Scene definition
##


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING

    # base for robot
    base = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Base",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.85, 0.4, 0.125], rot=[1.0, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_LOCAL_DIR}/Robots/Fairino/750base.usd",
            scale=(0.001, 0.001, 0.001),
        ),
    )
    # table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[1.0, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_LOCAL_DIR}/Samples/Examples/FrankaNutBolt/SubUSDs/Shop_Table/Shop_Table.usd",
            scale=(0.01, 0.0145, 0.009),
        ),
    )
    # box
    box = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.15643, -0.50583, 0.74398], rot=[0.70711, 0, 0, -0.70711]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/carton_box/carton_box.usd",
            scale=(0.004, 0.004, 0.003),
        ),
    )
    # box2
    box2 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Box2",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.12, -0.1, 0.81739], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_LOCAL_DIR}/Props/KLT_Bin/small_KLT_visual_collision.usd",
            scale=(1, 1, 1),
        ),
    )
    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # tvcontrol_01 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_01",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.15756, -0.51724, 0.78], rot=[0.66038, 0.19426, -0.1904, 0.69994]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )

    # tvcontrol_02 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_02",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.16398, -0.44239, 0.79], rot=[0.69741, 0.11671, 0.45923, -0.53769]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvcontrol_03 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_03",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.13816, -0.6033, 0.77657], rot=[0.68497, -0.11637, 0.02054, 0.71893]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvcontrol_04 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_04",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.11685, -0.52781, 0.83633], rot=[0.77413, 0.12663, -0.55444, 0.278]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvcontrol_05 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_05",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.21, -0.557221, 0.83633], rot=[0.88234, -0.29523, 0.11629, 0.34756]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvcontrol_06 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_06",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.11223, -0.40235, 0.84352], rot=[0.854002, 0.28209, -0.09042, 0.45454]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvcontrol_07 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/tvcontrol_07",
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.07305, -0.50412, 0.89249], rot=[0.08555, -0.19008, 0.97789, -0.01663]),
    #     spawn=UsdFileCfg(
    #         usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
    #         scale=(10, 10, 10),
    #         semantic_tags=[("class", "tvcontrol")],
    #     ),
    # )
    # tvconotrol2 = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/tvconotrol2",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[1.0, 0, 0, 0]),
    #     spawn=UsdFileCfg(
    #         usd_path="/home/user/IsaacLab/usd_test/TVConotrol3.usd",
    #         scale=(1, 1, 1),
    #     ),
    # )

##
# MDP settings
##
@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


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
        cube_positions = ObsTerm(func=mdp.cube_positions_in_world_frame)
        cube_orientations = ObsTerm(func=mdp.cube_orientations_in_world_frame)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class RGBCameraPolicyCfg(ObsGroup):
        """Observations for policy group with RGB images."""

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
    rgb_camera: RGBCameraPolicyCfg = RGBCameraPolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    cube_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("tvcontrol")}
    )

    success = DoneTerm(
        func=mdp.task_done, params={"object_cfg": SceneEntityCfg("tvcontrol")}
    )


@configclass
class PickEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the stacking environment."""

    seed = 1
    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5, replicate_physics=False)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    terminations: TerminationsCfg = TerminationsCfg()

    # Unused managers
    commands = None
    rewards = None
    events = None
    curriculum = None

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 30.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = 2

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

