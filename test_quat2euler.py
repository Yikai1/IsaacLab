
from scipy.spatial.transform import Rotation as R
import numpy as np
import torch
# 四元数，格式为 [x, y, z, w]
quat = [-0.56422, 0.1801, -0.09114, 0.80057]  # 替换为你的实际值，例如 [0, 0, 0.7071, 0.7071]

# 创建 Rotation 对象
r = R.from_quat(quat)

# 转换为欧拉角（单位：角度），默认顺序为 'xyz'
euler_angles = r.as_euler('zyx', degrees=True)

print("Euler angles (degrees):", euler_angles)


import isaaclab.utils.math as math_utils
quat_tensor = torch.tensor([
    [0.7, 0.0, 0.0, 0.7],  # 单位四元数（无旋转）
    [0.80057, -0.56422, 0.1801, -0.09114],
    [-0.78071, 0.59139, 0.03213, 0.19928]
], dtype=torch.float32)
roll, pitch, yaw = math_utils.euler_xyz_from_quat(quat_tensor)
print(roll * 180 / torch.pi, pitch * 180 / torch.pi, yaw * 180 / torch.pi)
