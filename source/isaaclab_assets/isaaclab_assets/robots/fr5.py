"""Configuration for the TCL frcobot.

The following configurations are available:

* :obj:`FR5_CFG`: FR5 robot without  gripper
* :obj:`FR5_DH_CFG`: FR5 robot with dh_pgc140 gripper

Reference: https://github.com/FAIR-INNOVATION/frcobot_ros/tree/master
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR, ISAAC_LOCAL_DIR

##
# Configuration
##
FR5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_LOCAL_DIR}/Robots/Fairino/fairino5_v6/fairino5_v6.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "j1": 0.0,
            "j2": -1.712,
            "j3": 1.712,
            "j4": -1.712,
            "j5": -1.712,
            "j6": -0.610,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["j[1-6]"],
            velocity_limit=2.175,
            effort_limit=87.0,
            stiffness=800.0,
            damping=4.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


FR5_DH_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_LOCAL_DIR}/Robots/Fairino/fr5_dh_pgc140/fr5_dh_pgc140.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "j1": -0.244528,
            "j2": -1.776268,
            "j3": 1.695729,
            "j4": -1.489571,
            "j5": -1.570796,
            "j6": -0.244528,
            "finger1_joint": 0.0,
            "finger2_joint": 0.0,
        },
        pos=(0.745, -0.514, 0.7525),
        rot=(1.0, 0, 0, 0),
    ),
    actuators={
        "fr_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["j[1-4]"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "fr_forearm": ImplicitActuatorCfg(
            joint_names_expr=["j[5-6]"],
            effort_limit_sim=12.0,
            velocity_limit_sim=2.61,
            stiffness=80.0,
            damping=4.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["finger[1-2]_joint"],
            effort_limit_sim=200.0,
            velocity_limit_sim=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

"""Configuration of Fr5 robot."""


FR5_HIGH_PD_CFG = FR5_DH_CFG.copy()
FR5_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
FR5_HIGH_PD_CFG.actuators["fr_shoulder"].stiffness = 400.0
FR5_HIGH_PD_CFG.actuators["fr_shoulder"].damping = 80.0
FR5_HIGH_PD_CFG.actuators["fr_forearm"].stiffness = 400.0
FR5_HIGH_PD_CFG.actuators["fr_forearm"].damping = 80.0
"""Configuration of Fr5 robot with stiffer PD control.
This configuration is useful for task-space control using differential IK.
"""
