from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
# my_world.scene.add_default_ground_plane()
env_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

add_reference_to_stage(usd_path=env_path, prim_path="/World/Environment")
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

# 第一台机械臂
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
gripper_1 = ParallelGripper(
    end_effector_prim_path="/World/Franka_1/panda_rightfinger",
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.05, 0.05]),
    joint_closed_positions=np.array([0.02, 0.02]),
    action_deltas=np.array([0.01, 0.01]),
)
franka_1 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka_1",
        name="franka_1",
        end_effector_prim_name="panda_rightfinger",
        gripper=gripper_1,
    )
)
# 第二台机械臂
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
gripper_2 = ParallelGripper(
    end_effector_prim_path="/World/Franka_2/panda_rightfinger",
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.05, 0.05]),
    joint_closed_positions=np.array([0.02, 0.02]),
    action_deltas=np.array([0.01, 0.01]),
)
franka_2 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka_2",
        name="franka_2",
        end_effector_prim_name="panda_rightfinger",
        gripper=gripper_2,
    )
)
# 左边机械臂
franka_1.set_world_pose(position=np.array([0.0, 0.0, 0.0]), orientation=[1, 0, 0, 0])
# 右边机械臂
franka_2.set_world_pose(position=np.array([0.0, 1.0, 0.0]), orientation=[1, 0, 0, 0])

cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.3, 0.5, 0.3]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([0, 0, 1]),
    )
)
camera = Camera(
    prim_path="/World/camera",
    position=np.array([5.0, 0.5, 3.5]),
    frequency=20,
    resolution=(1920, 1080),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 56, 90]), degrees=True)
)
# camera = Camera(
#     prim_path="/World/camera",
#     position=np.array([0.0, 0.0, 25.0]),
#     frequency=20,
#     resolution=(256, 256),
#     orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)
# )

my_world.scene.add_default_ground_plane()
franka_1.gripper.set_default_state(franka_1.gripper.joint_opened_positions)
franka_2.gripper.set_default_state(franka_2.gripper.joint_opened_positions)
my_world.reset()
camera.initialize()

controller_1 = PickPlaceController(
    name="controller_1", gripper=franka_1.gripper, robot_articulation=franka_1
)
controller_2 = PickPlaceController(
    name="controller_2", gripper=franka_2.gripper, robot_articulation=franka_2
)
controller_1.reset()
controller_2.reset()

articulation_controller_1 = franka_1.get_articulation_controller()
articulation_controller_2 = franka_2.get_articulation_controller()


i = 0
camera.add_motion_vectors_to_frame()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            controller_1.reset()
            controller_2.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions_1 = controller_1.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([0.5, 0.5, 0.0515 / 2.0]),
            current_joint_positions=franka_1.get_joint_positions(),
            end_effector_offset=np.array([0.0, 0.01, 0.0]),
        )
        actions_2 = controller_2.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([-0.5, -0.5, 0.0515 / 2.0]),
            current_joint_positions=franka_2.get_joint_positions(),
            end_effector_offset=np.array([0.0, -0.01, 0.0]),
        )
        # imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        # plt.show()
        # print(camera.get_current_frame()["motion_vectors"])
        articulation_controller_1.apply_action(actions_1)
        articulation_controller_2.apply_action(actions_2)
        rgba = camera.get_rgba()  # shape: (H, W, 4), float32 in [0, 1]
    # contacts = my_world.physics_interface.get_contact_report()
    # if contacts:
    #     print("Collision detected!")
    
    if args.test is True:
        break
    
simulation_app.close()
# video_writer.release() 