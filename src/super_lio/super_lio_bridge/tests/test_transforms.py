"""transforms.py 的数值单测（不依赖 ROS，可直接 python3 运行）。"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from super_lio_bridge import transforms as tf  # noqa: E402


def approx(a, b, tol=1e-9):
    return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=tol)


def main():
    rng = np.random.default_rng(20261016)
    failures = []

    # 1) 四元数 <-> 旋转矩阵往返
    worst = 0.0
    for _ in range(2000):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = tf.quat_to_matrix(q[0], q[1], q[2], q[3])
        # 正交性与行列式
        assert approx(R @ R.T, np.eye(3), 1e-9), 'R 非正交'
        assert abs(np.linalg.det(R) - 1.0) < 1e-9, 'det(R) != 1'
        q2 = tf.matrix_to_quat(R)
        # 允许 q 与 -q 表示同一旋转
        err = min(np.linalg.norm(q - q2), np.linalg.norm(q + q2))
        worst = max(worst, err)
    print(f'1) 四元数往返最大误差: {worst:.3e}')
    if worst > 1e-9:
        failures.append('四元数往返误差过大')

    # 2) 刚体变换求逆
    worst = 0.0
    for _ in range(2000):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        T = tf.make_T(tf.quat_to_matrix(*q), rng.normal(size=3))
        prod = T @ tf.invert_T(T)
        worst = max(worst, float(np.abs(prod - np.eye(4)).max()))
    print(f'2) T @ inv(T) 与单位阵最大偏差: {worst:.3e}')
    if worst > 1e-9:
        failures.append('求逆不正确')

    # 3) rpy 往返
    worst = 0.0
    for _ in range(2000):
        rpy = rng.uniform(-np.pi / 2 * 0.9, np.pi / 2 * 0.9, size=3)
        R = tf.rpy_to_matrix(*rpy)
        rpy2 = np.array(tf.rpy_of(tf.make_T(R, [0, 0, 0])))
        worst = max(worst, float(np.abs(rpy - rpy2).max()))
    print(f'3) rpy 往返最大误差: {worst:.3e}')
    if worst > 1e-9:
        failures.append('rpy 往返不正确')

    # 4) 已知算例：先绕 Z 转 90°，再沿 X 平移 1 —— 对原点的影响应等于 (1,0,0)
    T = tf.make_T(tf.rpy_to_matrix(0, 0, np.pi / 2), [1.0, 0.0, 0.0])
    origin = np.array([0.0, 0.0, 0.0, 1.0])
    got = (T @ origin)[:3]
    print(f'4) 已知算例 T·origin = {np.round(got, 9)}（应为 [1,0,0]）')
    if not approx(got, [1.0, 0.0, 0.0]):
        failures.append('已知算例不对')

    # 5) 组合语义：T_a_b 表示「把 b 系的点变换到 a 系」，
    #    那么 T_world_base = T_world_imu * T_imu_base
    yaw90 = tf.rpy_to_matrix(0, 0, np.pi / 2)
    T_world_imu = tf.make_T(yaw90, [2.0, 0.0, 0.0])
    T_imu_base = tf.make_T(np.eye(3), [1.0, 0.0, 0.0])
    T_world_base = T_world_imu @ T_imu_base
    got = T_world_base[:3, 3]
    print(f'5) 组合后 base 在 world 的位置 = {np.round(got, 9)}（应为 [2,1,0]）')
    if not approx(got, [2.0, 1.0, 0.0]):
        failures.append('变换组合的语义不对')

    # 6) 相对运动：T_odom_base = inv(T_boot) * T_world_base
    T_odom_base = tf.invert_T(T_world_imu) @ T_world_base
    got = T_odom_base[:3, 3]
    print(f'6) 相对 boot 的 base 位置 = {np.round(got, 9)}（应为 [1,0,0]）')
    if not approx(got, [1.0, 0.0, 0.0]):
        failures.append('相对运动计算不对')

    # 7) 欧拉角约定必须与 ROS/URDF 一致：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    #    这条独立于任何库、手工构造，用来防止换库时 from_euler 的约定被用反
    #    （约定反了是旋转顺序不同，矩阵看着"差不多"，但 TF 会静默地歪掉）。
    def manual_rpy(r, p, y):
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    worst = 0.0
    for _ in range(500):
        rpy = rng.uniform(-np.pi * 0.9, np.pi * 0.9, size=3)
        worst = max(worst, float(np.abs(tf.rpy_to_matrix(*rpy) - manual_rpy(*rpy)).max()))
    print(f'7) rpy 与手工 Rz@Ry@Rx 的最大偏差: {worst:.3e}')
    if worst > 1e-12:
        failures.append('欧拉角约定与 ROS/URDF 不一致')

    # 8) 约定一致性抽查：yaw=90° 应把 x 轴转到 y 轴
    R = tf.rpy_to_matrix(0.0, 0.0, np.pi / 2)
    got = R @ np.array([1.0, 0.0, 0.0])
    print(f'8) yaw=90° 作用于 x 轴 = {np.round(got, 9)}（应为 [0,1,0]）')
    if not approx(got, [0.0, 1.0, 0.0], 1e-12):
        failures.append('yaw 方向不对')

    print()
    if failures:
        for f in failures:
            print('FAIL -', f)
        return 1
    print('=== transforms 全部单测通过 ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
