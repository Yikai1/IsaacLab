# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from sched import Event
from isaaclab.assets import RigidObjectCfg, AssetBaseCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
# from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAAC_LOCAL_DIR
from isaaclab_tasks.manager_based.manipulation.cluttered_pick import mdp
from isaaclab_tasks.manager_based.manipulation.cluttered_pick.mdp import fr_pick_events
from isaaclab_tasks.manager_based.manipulation.cluttered_pick.pick_env_cfg import PickEnvCfg
##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.fr5 import FR5_DH_CFG  # isort: skip
from isaaclab.utils.assets import ISAAC_LOCAL_DIR


@configclass
class EventCfg:
    """Configuration for events."""

    # reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    
    init_fr_arm_pose = EventTerm(
        func=fr_pick_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0, -1.712, 1.712, -1.712, -1.712, -0.61, 0.0, 0.0],
        },
    )
    
    # init_tvcontrol = EventTerm(
    #     func=fr_pick_events.set_default_object_pose,
    #     mode="reset",
    #     params={
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol_01"), SceneEntityCfg("tvcontrol_02"), SceneEntityCfg("tvcontrol_03"), SceneEntityCfg("tvcontrol_04"), SceneEntityCfg("tvcontrol_05"), SceneEntityCfg("tvcontrol_06"), SceneEntityCfg("tvcontrol_07"), SceneEntityCfg("tvcontrol_08"), SceneEntityCfg("tvcontrol_09")],
    #     },
    # )

    randomize_fr_joint_state = EventTerm(
        func=fr_pick_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_tvcontrol_positions = EventTerm(
        func=fr_pick_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.13, 0.18), "y": (-0.43, -0.57), "z": (0.88, 0.88), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("tvcontrol")],
        },
    )
    # randomize_tvcontrol2_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2"), SceneEntityCfg("tvcontrol2_01"), SceneEntityCfg("tvcontrol2_02"), SceneEntityCfg("tvcontrol2_03"), SceneEntityCfg("tvcontrol2_04"), SceneEntityCfg("tvcontrol2_05")],
    #     },
    # )
    # randomize_tvcontrol2_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2")],
    #     },
    # )
    # randomize_tvcontrol3_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2_01")],
    #     },
    # )
    # randomize_tvcontrol4_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2_02")],
    #     },
    # )
    # randomize_tvcontrol5_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2_03")],
    #     },
    # )
    # randomize_tvcontrol6_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2_04")],
    #     },
    # )
    # randomize_tvcontrol7_positions = EventTerm(
    #     func=fr_pick_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.13, 0.18), "y": (-0.45, -0.55), "z": (0.80, 0.80), "roll": (-3.14, 3.14), "yaw": (-0.5236, 0.5236), "pitch": (-2.0944, -1.0472), },
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("tvcontrol2_05")],
    #     },
    # )


@configclass
class FR5TVControlPickEnvCfg(PickEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set events
        self.events = EventCfg()

        # Set FR5 as robot
        self.scene.robot = FR5_DH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        # Add semantics to table
        self.scene.table.spawn.semantic_tags = [("class", "table")]

        # Add semantics to ground
        # self.scene.plane.semantic_tags = [("class", "ground")]

        # Set actions for the specific robot type (FR5)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["j[1-6]"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger1_joint", "finger2_joint"],
            open_command_expr={"finger1_joint": 0.0, "finger2_joint": 0.0},
            close_command_expr={"finger1_joint": 0.025, "finger2_joint": 0.025},
        )

        # Rigid body properties of each cube
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Set each stacking cube deterministically
        # self.scene.cube = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Cube",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.05, 0.3, 0.75], rot=[1, 0, 0, 0]),
        #     spawn=sim_utils.CuboidCfg(
        #         size=(0.03, 0.03, 0.03),
        #         rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        #         mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
        #         collision_props=sim_utils.CollisionPropertiesCfg(
        #             collision_enabled=True,
        #             contact_offset=1e-3,
        #             rest_offset=0.0,
        #             torsional_patch_radius=1e-3,
        #             min_torsional_patch_radius=1e-3,
        #         ),
        #         physics_material=sim_utils.spawners.RigidBodyMaterialCfg(
        #             static_friction=1.5,
        #             dynamic_friction=1.5,
        #             restitution=0.0,
        #         ),
        #         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0), metallic=0.2),
        #     )
        # )
        self.scene.tvcontrol = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.15, -0.5, 1.0], rot=[1.0, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                rigid_props=cube_properties,
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        # self.scene.tvconotrol2 = AssetBaseCfg(
        #     prim_path="{ENV_REGEX_NS}/tvconotrol2",
        #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[1.0, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path="/home/user/IsaacLab/usd_test/TVConotrol3.usd",
        #         scale=(1, 1, 1),
        #     ),
        # )
        self.scene.tvcontrol_01 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_01",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.15756, -0.51724, 0.75502], rot=[0.66038, 0.19426, -0.1904, 0.69994]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_02 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_02",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.16398, -0.43959, 0.76537], rot=[0.71521, 0.30339, 0.29654, -0.55542]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_03 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_03",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.13816, -0.6033, 0.75746], rot=[0.68497, -0.11637, 0.02054, 0.71893]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_04 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_04",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.07421, -0.52781, 0.78672], rot=[0.92019, 0.02, -0.3906, -0.02408]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_05 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_05",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.21, -0.57233, 0.82496], rot=[0.88234, -0.29523, 0.11629, 0.34756]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_06 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_06",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.18249, -0.45229, 0.8189], rot=[0.854002, 0.28209, -0.09042, 0.45454]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_07 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_07",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.12012, -0.58242, 0.83405], rot=[0.60291, -0.36946, 0.61543, 0.3482]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_08 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_08",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.1832, -0.50511, 0.81474], rot=[0.82902, -0.00235, 0.01211, 0.55909]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        self.scene.tvcontrol_09 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/tvcontrol_09",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.18103, -0.4093, 0.80319], rot=[0.70707, 0.00617, -0.17705, 0.68459]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                semantic_tags=[("class", "tvcontrol")],
            ),
        )
        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wrist3_Link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/finger1_link",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.03),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/finger2_link",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.03),
                    ),
                ),
            ],
        )
