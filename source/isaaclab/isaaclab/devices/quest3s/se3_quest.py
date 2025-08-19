from oculus_reader import OculusReader
import numpy as np
np.set_printoptions(precision=4)
from ..device_base import DeviceBase
import threading
import time
from collections.abc import Callable
from scipy.spatial.transform import Rotation

class Se3Quest(DeviceBase):
    """A quest3s controller for sending SE(3) commands as delta poses."""
    def __init__(self, pos_sensitivity: float = 0.4, rot_sensitivity: float = 0.8):
        """Initialize the space-mouse layer.

        Args:
            pos_sensitivity: Magnitude of input position command scaling. Defaults to 0.4.
            rot_sensitivity: Magnitude of scale input rotation commands scaling. Defaults to 0.8.
        """
        # store inputs
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        # acquire device interface
        self.reader = OculusReader()
        # read rotations
        self._read_rotation = False

        # command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)
        # dictionary for additional callbacks
        self._additional_callbacks = dict()
        # run a thread for listening to device updates
        self._thread = threading.Thread(target=self._run_device)
        self._thread.daemon = True
        self._thread.start()
    
    """
    Operations
    """

    def reset(self):
        # default flags
        self._close_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)

    def add_callback(self, key: str, func: Callable):
        """Add additional functions to bind keyboard.

        A list of available keys are present in the
        `carb documentation <https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html>`__.

        Args:
            key: The keyboard button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool]:
        """Provides the result from spacemouse event state.

        Returns:
            A tuple containing the delta pose command and gripper commands.
        """
        rot_vec = Rotation.from_euler("XYZ", self._delta_rot).as_rotvec()
        # if new command received, reset event flag to False until keyboard updated.
        return np.concatenate([self._delta_pos, rot_vec]), self._close_gripper
    
    
    def _run_device(self):
        """Listener thread that keeps pulling new messages."""
        # config
        vr_count = 0
        # last_flag = False
        # last_handle_button = False
        # gripper_state = False
        # keep running
        # 还有夹爪没写，还没有转四元数格式
        while True:
            # read the device data
            self.poses, self.buttons = self.reader.get_transformations_and_buttons()
            if 'r' in self.poses:
                self.current_vr_transform_r, self.trigger, self.trigger_continuous, self.handle_button = self.poses['r'], self.buttons['RTr'], self.buttons['rightTrig'][0], self.buttons['RG']
                print("trigger:",self.trigger)
                print("trigger_continuous:",self.trigger_continuous)
                print("handle_button:",self.handle_button)
            
            if self.current_vr_transform_r is not None:
                # readings from 6-DoF sensor
                x = self.current_vr_transform_r[0, 3]
                y = self.current_vr_transform_r[1, 3]
                z = self.current_vr_transform_r[2, 3]

                x,y,z = z,x,y
                # 提取旋转矩阵部分
                rotation_matrix = self.current_vr_transform_r[:3, :3]
                # print("rotation_matrix:",rotation_matrix)

                if np.count_nonzero(rotation_matrix) == 0:
                    print("该矩阵全为 0")
                    continue

                # 将旋转矩阵转换为欧拉角（ZYX顺序）
                euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('zyx', degrees=False)
                yaw, pitch, roll = euler_angles

            # print("yawpitchroll",yaw* 180 / np.pi,pitch* 180 / np.pi,roll* 180 / np.pi)

                self.delta_pos = [0.0,0.0,0.0,0.0,0.0,0.0]

                if vr_count == 0:
                    start_pos = [x, y, z, roll, pitch, yaw]
                    last_x = x
                    last_y = y
                    last_z = z
                    last_roll = roll
                    last_pitch = pitch
                    last_yaw = yaw
                    vr_count += 1

                self.delta_pos = [x - start_pos[0], y - start_pos[1], z - start_pos[2], roll - start_pos[3],
                                pitch - start_pos[4],
                                yaw - start_pos[5]]
                                #print("delta_pos:", np.array(delta_pos))
                if abs(x - last_x) > 0.5 or abs(y - last_y) > 0.5 or abs(z - last_z) > 0.5:
                    print("xyz exceed limits")
                    start_pos = [x, y, z, roll, pitch, yaw]
                if abs(roll - last_roll) > 20/180 * np.pi or abs(pitch - last_pitch) > 20/180 * np.pi or abs(yaw - last_yaw) > 20/180 * np.pi:
                    print("roll pitch yaw exceed limits")
                    start_pos = [x, y, z, roll, pitch, yaw]
                delta_pos = [x - start_pos[0], y - start_pos[1], z - start_pos[2], roll - start_pos[3], pitch - start_pos[4],
                             yaw - start_pos[5]]
                last_x = x
                last_y = y
                last_z = z
                last_roll = roll
                last_pitch = pitch
                last_yaw = yaw