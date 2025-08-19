# from isaacsim import SimulationApp

# simulation_app = SimulationApp({"headless": False})

import argparse
from scipy.spatial.transform import Rotation as R
import numpy as np
import copy
import time

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the differential IK controller.")
# parser.add_argument("--robot", type=str, default="franka_panda", help="Name of the robot.")
parser.add_argument("--robot", type=str, default="ur10", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# # append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# # launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot_setup.assembler")
from isaacsim.robot_setup.assembler import RobotAssembler,AssembledRobot
"""Rest everything follows."""

import torch
# from omni.physx.scripting import create_material, apply_material_to_prim
import isaaclab.sim as sim_utils
# from isaaclab.sim.spawners import materials
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAAC_LOCAL_DIR
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
# ##
# # Pre-defined configs
# ##
from isaaclab_assets.robots.fr5 import FR5_CFG
from isaaclab_assets.gripper.dh import DH_PGC140_CFG
# def get_init_rot():
#     """Get the initial rotation of the robot."""
#     x,y,z,w = R.from_euler("xyz", [0, 0, 0], degrees=True).as_quat()
#     quat = (w,x,y,z)
#     return quat


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # mount
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_LOCAL_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd", scale=(1.0, 1.0, 1.0)
        ),
    )

    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.03, 0.03, 0.03),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=1e-3,
                rest_offset=0.0,
                torsional_patch_radius=1e-3,
                min_torsional_patch_radius=1e-3
            ),
            physics_material=sim_utils.spawners.RigidBodyMaterialCfg(
                static_friction=1.5,
                dynamic_friction=1.5,
                restitution=0.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0), metallic=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
        ),
    )

    robot = DH_PGC140_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")  # type: ignore
    robot.init_state.pos = (0.0, 0.0, 0.15)
    robot.init_state.rot = (0.0, 1.0, 0.0, 0.0)

    # # -- Robot
    # robot = FR5_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    # # -- Gripper
    # my_gripper_cfg = DH_PGC140_CFG.replace(prim_path="{ENV_REGEX_NS}/gripper")

    # robot_assembler = RobotAssembler()
    # assembled_robot = robot_assembler.assemble_articulations(base_robot_path="{ENV_REGEX_NS}/robot",
    #                                                    attach_robot_path="{ENV_REGEX_NS}/gripper",
    #                                                    base_robot_mount_frame="/wrist3_Link",
    #                                                    attach_robot_mount_frame="/base_link",
    #                                                    fixed_joint_offset=np.array([0.0, 0.0, 0.1]),
    #                                                    fixed_joint_orient=np.array([1.0, 0.0, 0.0, 0.0]),
    #                                                    mask_all_collisions=True,
    #                                                    single_robot=True)


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    count = 0
    sim_dt = sim.get_physics_dt()
    robot = scene["robot"]
    # robot_entity_cfg = SceneEntityCfg("robot", joint_names=['finger_joint', 'left_inner_knuckle_joint', 'right_inner_knuckle_joint', 'right_outer_knuckle_joint', 'left_inner_finger_joint', 'right_inner_finger_joint'])
    # Simulation loop

    while simulation_app.is_running():
        if count % 500 == 0:
            # reset counter
            count = 0
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.zeros_like(joint_pos)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")

        joint_pose = torch.tensor([[0.025,0.025]], device=robot.device)
        robot.set_joint_position_target(joint_pose)
        robot.write_data_to_sim()
        count += 1
        sim.step()


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])  # type: ignore
    # Design scene
    scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


def goal_pose_callback(data):
    print("Received goal pose")


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()