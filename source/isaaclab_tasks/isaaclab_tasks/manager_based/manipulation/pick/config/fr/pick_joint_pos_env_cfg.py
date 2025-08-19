# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
# from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAAC_LOCAL_DIR
from isaaclab_tasks.manager_based.manipulation.pick import mdp
from isaaclab_tasks.manager_based.manipulation.pick.mdp import fr_pick_events
from isaaclab_tasks.manager_based.manipulation.pick.pick_env_cfg import PickEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.fr5 import FR5_DH_CFG
from isaaclab.utils.assets import ISAAC_LOCAL_DIR

@configclass
class EventCfg:
    """Configuration for events."""

    init_fr_arm_pose = EventTerm(
        func=fr_pick_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0, -1.712, 1.712, -1.712, -1.712, -0.61, 0.0, 0.0],
        },
    )

    randomize_fr_joint_state = EventTerm(
        func=fr_pick_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_cube_positions = EventTerm(
        func=fr_pick_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.2), "y": (-0.30, -0.60), "z": (0.80, 0.80), "yaw": (-1, 1), "pitch": (1.5708, 1.5708), },
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("tvcontrol")],
        },
    )


@configclass
class FR5CubePickEnvCfg(PickEnvCfg):
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
        self.scene.plane.semantic_tags = [("class", "ground")]

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
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, -0.4, 0.9], rot=[0.5, 0.5, 0.5, 0.5]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_LOCAL_DIR}/Props/TCL/tvcontrol/tvcontrol.usd",
                scale=(10, 10, 10),
                rigid_props=cube_properties,
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
                    prim_path="{ENV_REGEX_NS}/Robot/wrist3_Link",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.0],
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/finger1_link",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/finger2_link",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.0),
                    ),
                ),
            ],
        )
