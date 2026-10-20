"""Super-LIO 桥接层 external 模式测试。

external 模式用于 tracking：
  Super-LIO 只当 odom 系里程计，map->odom 由 small_gicp_relocalization 发布。
验证：
  1. odom->base_footprint 直接等于 Super-LIO 的 world 位姿（不再绕第一帧锚定）
  2. 桥接层不发布 map->odom
  3. /registered_scan 的 frame_id = odom
  4. 桥接层不发布 /localization/tracking_ok（该判定归 small_gicp）
  5. /odom 的 frame_id 与内容正确
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
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

IMU_TO_BASE_Z = -0.32


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Probe(Node):

    def __init__(self):
        super().__init__('bridge_external_probe')
        self.pub = self.create_publisher(Odometry, '/lio/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/lio/cloud_world', 10)
        self.ok_values = []
        self.odom_msgs = []
        self.scan_frames = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/localization/tracking_ok',
            lambda m: self.ok_values.append(m.data), latched)
        self.create_subscription(
            Odometry, '/odom', lambda m: self.odom_msgs.append(m), 10)
        self.create_subscription(
            PointCloud2, '/registered_scan',
            lambda m: self.scan_frames.append(m.header.frame_id), 10)
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


def lookup(node, parent, child, timeout=0.3):
    try:
        t = node.tf_buffer.lookup_transform(
            parent, child, rclpy.time.Time(),
            rclpy.duration.Duration(seconds=timeout))
    except Exception as e:
        return None, str(e)
    tr = t.transform
    yaw = math.atan2(2.0 * (tr.rotation.w * tr.rotation.z),
                     1.0 - 2.0 * tr.rotation.z ** 2)
    return (tr.translation.x, tr.translation.y, tr.translation.z, yaw), None


def main():
    proc = subprocess.Popen(
        ['ros2', 'run', 'super_lio_bridge', 'tf_bridge', '--ros-args',
         '-p', 'odom_timeout:=0.8', '-p', 'localization_mode:=external'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid)

    rclpy.init()
    node = Probe()
    results = []

    try:
        node.hold(2.0)
        node.hold(2.0, pose=(1.0, 2.0, 0.5, math.pi / 2))

        ob, err = lookup(node, 'odom', 'base_footprint')
        print(f'1) odom->base_footprint = {None if ob is None else tuple(round(v, 4) for v in ob)}  {err or ""}')
        ok1 = (ob is not None and abs(ob[0] - 1.0) < 0.02 and abs(ob[1] - 2.0) < 0.02
               and abs(ob[2] - (0.5 + IMU_TO_BASE_Z)) < 0.02
               and abs(ob[3] - math.pi / 2) < 0.02)
        results.append(('odom->base 直接等于 world 位姿（未锚定）', ok1))

        mo, err = lookup(node, 'map', 'odom')
        print(f'2) map->odom = {None if mo is None else tuple(round(v, 4) for v in mo)}  {err or ""}')
        results.append(('桥接层不发布 map->odom', mo is None))

        node.hold(0.5)
        frames = set(node.scan_frames)
        print(f'3) /registered_scan frame_id = {frames}')
        results.append(('/registered_scan 使用 odom 帧', frames == {'odom'}))

        print(f'4) tracking_ok 消息数 = {len(node.ok_values)}')
        results.append(('external 模式不发 tracking_ok', len(node.ok_values) == 0))

        if node.odom_msgs:
            last = node.odom_msgs[-1]
            ok5 = (last.header.frame_id == 'odom'
                   and abs(last.pose.pose.position.x - ob[0]) < 0.05
                   and abs(last.pose.pose.position.y - ob[1]) < 0.05)
        else:
            ok5 = False
        results.append(('/odom 使用 odom 帧且与 TF 一致', ok5))
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
