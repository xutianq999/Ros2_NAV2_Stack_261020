"""Super-LIO 桥接层的集成测试。

喂合成的 /lio/odom（模拟 Super-LIO 在 world 系的位姿），验证：
  1. map->odom 被锚定在第一帧位姿，且之后保持不变
  2. odom->base_footprint 正确反映相对运动（含 imu->base 的静态外参）
  3. /odom 与 TF 一致
  4. 位姿停止更新后，/localization/tracking_ok 变 false（安全层据此锁住自动驾驶）
  5. 点云被转发到 /registered_scan
"""

import math
import os
import signal
import struct
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

BRIDGE_TIMEOUT = {'odom_timeout': 0.8}


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Probe(Node):

    def __init__(self):
        super().__init__('bridge_probe')
        self.pub = self.create_publisher(Odometry, '/lio/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/lio/cloud_world', 10)
        self.ok_values = []
        self.status = []
        self.odom_msgs = []
        self.scan_count = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/localization/tracking_ok',
                                 lambda m: self.ok_values.append((time.monotonic(), m.data)), latched)
        self.create_subscription(String, '/localization/tracking_status',
                                 lambda m: self.status.append(m.data), 10)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.odom_msgs.append(m), 10)
        self.create_subscription(PointCloud2, '/registered_scan',
                                 self._on_scan, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cloud = PointCloud2()
        cloud.height = 1
        cloud.width = 3
        cloud.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z'))]
        cloud.point_step = 12
        cloud.row_step = 36
        cloud.is_dense = True
        cloud.data = struct.pack('<fffffffff', 1, 0, 0, 2, 0, 0, 3, 0, 0)
        self.cloud = cloud

    def _on_scan(self, msg):
        self.scan_count += 1

    def send_pose(self, x, y, z, yaw):
        qx, qy, qz, qw = yaw_quat(yaw)
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'world'
        od.child_frame_id = 'imu'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.position.z = z
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.pub.publish(od)
        self.cloud.header.stamp = od.header.stamp
        self.cloud_pub.publish(self.cloud)

    def hold(self, seconds, pose=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if pose:
                self.send_pose(*pose)
            rclpy.spin_once(self, timeout_sec=0.02)


def lookup(node, parent, child):
    try:
        t = node.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
    except Exception as e:
        return None, str(e)
    tr = t.transform
    yaw = math.atan2(2.0 * (tr.rotation.w * tr.rotation.z), 1.0 - 2.0 * tr.rotation.z ** 2)
    return (tr.translation.x, tr.translation.y, tr.translation.z, yaw), None


def main():
    proc = subprocess.Popen(
        ['ros2', 'run', 'super_lio_bridge', 'tf_bridge', '--ros-args',
         '-p', 'odom_timeout:=0.8'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid)

    rclpy.init()
    node = Probe()
    results = []

    try:
        node.hold(2.0)

        # 第一帧：world 系下 (1, 2, 0.5)，yaw=90°
        boot = (1.0, 2.0, 0.5, math.pi / 2)
        node.hold(2.0, pose=boot)

        mo, err1 = lookup(node, 'map', 'odom')
        ob, err2 = lookup(node, 'odom', 'base_footprint')
        mb, err3 = lookup(node, 'map', 'base_footprint')
        print(f'1) map->odom = {None if mo is None else tuple(round(v,4) for v in mo)}  {err1 or ""}')
        print(f'   odom->base_footprint = {None if ob is None else tuple(round(v,4) for v in ob)}  {err2 or ""}')
        print(f'   map->base_footprint = {None if mb is None else tuple(round(v,4) for v in mb)}  {err3 or ""}')

        # 真正的正确性判据是**组合不变式**：map->base 必须等于
        # T_world_imu ⊗ T_imu_base（Super-LIO 位姿 + imu->base 外参）。
        # 单独看中间那两个变换容易被误判：
        #   map->odom 的 z 是 0.5-0.32=0.18（外参已并入 T_boot），
        #   odom->base 在启动点是单位阵（T_odom_base = inv(T_boot) ⊗ T_world_base 相消）。
        ok1 = (mb is not None and abs(mb[0] - boot[0]) < 0.02 and abs(mb[1] - boot[1]) < 0.02
               and abs(mb[2] - (boot[2] - 0.32)) < 0.02 and abs(mb[3] - boot[3]) < 0.02)
        results.append(('组合不变式：map->base == world 位姿 ⊗ imu->base', ok1))

        # map->odom 锚定在第一帧的 base 位姿（含外参带来的 z 偏移）
        ok2 = mo is not None and abs(mo[0] - 1.0) < 0.02 and abs(mo[1] - 2.0) < 0.02 \
            and abs(mo[2] - 0.18) < 0.02 and abs(mo[3] - math.pi / 2) < 0.02
        results.append(('map->odom 锚定第一帧位姿（含外参 z 偏移）', ok2))

        # 启动点处无相对运动 -> 单位阵
        ok2b = ob is not None and abs(ob[0]) < 0.02 and abs(ob[1]) < 0.02 \
            and abs(ob[2]) < 0.02 and abs(ob[3]) < 0.02
        results.append(('启动点 odom->base_footprint 为单位阵', ok2b))

        # 第二帧：world 系下前进 1m、yaw 不变 —— 相对运动应在 odom 里体现为 +1m
        moved = (2.0, 2.0, 0.5, math.pi / 2)
        node.hold(2.0, pose=moved)
        ob2, _ = lookup(node, 'odom', 'base_footprint')
        print(f'2) 前进 1m 后 odom->base = {None if ob2 is None else tuple(round(v,4) for v in ob2)}')
        # boot 处 yaw=90°，world 系 +x 前进 = odom 系下的 +y? 不：odom 相对 boot，
        # 位移 (1,0,0) 经 inv(T_boot) 旋转 -90° 后为 (0,-1,0)
        ok3 = ob2 is not None and abs(ob2[0] - 0.0) < 0.02 and abs(ob2[1] + 1.0) < 0.02
        results.append(('odom 中的相对运动正确（world +x 对应 odom -y）', ok3))

        # map->odom 保持不变
        mo2, _ = lookup(node, 'map', 'odom')
        ok4 = mo2 is not None and all(abs(a - b) < 1e-6 for a, b in zip(mo, mo2))
        results.append(('map->odom 保持不变', ok4))

        # /odom 与 TF 一致（比较最后一帧）
        node.hold(0.5)
        if node.odom_msgs:
            last = node.odom_msgs[-1]
            ok5 = (abs(last.pose.pose.position.x - ob2[0]) < 0.05 and
                   abs(last.pose.pose.position.y - ob2[1]) < 0.05)
        else:
            ok5 = False
        results.append(('/odom 与 TF 一致', ok5))

        # 点云转发
        results.append(('/registered_scan 收到点云', node.scan_count > 0))
        print(f'3) /registered_scan 收到 {node.scan_count} 帧')

        # tracking_ok 应为 true
        ok_now = node.ok_values[-1][1] if node.ok_values else None
        results.append(('tracking_ok = true（位姿正常）', ok_now is True))

        # 停止发布 -> 超时后应变 false
        node.hold(2.5, pose=None)
        ok_after = node.ok_values[-1][1] if node.ok_values else None
        print(f'4) 停止发布 2.5s 后 tracking_ok = {ok_after}')
        results.append(('位姿超时后 tracking_ok = false', ok_after is False))
        if node.status:
            print(f'   状态: {node.status[-1]}')

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=6)
        except Exception:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    print()
    failed = [r for r in results if not r[1]]
    for name, ok in results:
        print(f'{"PASS" if ok else "FAIL"}  {name}')
    if failed:
        print('\n--- 桥接日志 ---')
        print((proc.stdout.read() if proc.stdout else '')[-1500:])
        return 1
    print(f'\n=== 全部 {len(results)} 项通过 ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
