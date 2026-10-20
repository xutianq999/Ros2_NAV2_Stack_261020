"""SE(3) 变换工具。

旋转部分（四元数 / 欧拉角 / 旋转矩阵互转）用 **scipy**：这里正是最容易出微妙错误
的地方（万向锁、近 180°、四元数符号约定），`scipy.spatial.transform.Rotation` 是
久经考验的实现，而且本环境已经装了 scipy，不增加依赖成本。
平移与 4x4 组合只是线性代数，用 numpy 直接写更直白。

⚠ 欧拉角约定（关键）：本文件用 `from_euler('xyz', [roll, pitch, yaw])`，scipy 里
小写 'xyz' 表示绕**固定轴**依次 x→y→z 旋转，等价于矩阵
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
这正是 ROS / URDF 的 rpy 约定。tests/ 里有一条独立于任何库的手工断言
（直接和 Rz@Ry@Rx 比）来把这条钉死 —— 约定搞反会让 TF 静默地歪掉。
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 -> 四元数 [x, y, z, w]（与 ROS 的顺序一致）。"""
    return Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS/URDF 约定的 rpy -> 旋转矩阵，即 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()


def rpy_of(T: np.ndarray) -> tuple:
    """4x4 变换的旋转部分 -> (roll, pitch, yaw)，与 rpy_to_matrix 互逆。"""
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
    return float(rpy[0]), float(rpy[1]), float(rpy[2])


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    """刚体变换求逆：R^T 与 -R^T t（比 np.linalg.inv 更快也更准）。"""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def quat_of(T: np.ndarray) -> np.ndarray:
    return matrix_to_quat(T[:3, :3])
