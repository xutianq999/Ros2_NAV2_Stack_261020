"""Super-LIO -> Nav2 桥接。

## 为什么需要这一层

Super-LIO 是**单坐标系**的 LIO：它只广播一个 TF `world -> imu`，并把位姿发在
`/lio/odom`（frame_id 为 `world`）。而 Nav2 要的是三段式
`map -> odom -> base_footprint`（局部代价地图以 `odom` 为参考做滚动窗口）。

它的重定位模式下，`world` 帧**就是先验地图的帧** —— 也就是说它给的是绝对定位，
没有"里程计 + 单独修正"两段。所以本节点做的映射是：

    odom -> base_footprint  =  相对启动位姿的运动（连续、无跳变，给局部代价地图用）
    map  -> odom            =  启动那一刻在 map 中的位姿（常量）

这样 map 系下的位姿 = 绝对定位结果（正确），而 odom 系保持连续。要注意：单坐标系
LIO 的漂移会直接体现在 map 系位姿上，没有任何独立的机制去纠正它 —— 这是它的固有
特性，不是本节点的缺陷。

## 为什么只走话题

本节点只通过 ROS 话题/TF 与 Super-LIO 交互，**不 include 它的头文件、不链接它的库**。
Super-LIO 是 GPLv3，这样桥接层就是独立的程序，其许可不受影响。也正因如此，本包
编译时不需要 Super-LIO 存在。

## 健康度是弱信号（重要）

`/localization/tracking_ok` 必须有人发 —— 安全层的看门狗默认盯这个话题，没人发
自动驾驶会被永久锁住。但 Super-LIO 内部虽有 `flg_init_` / `flg_get_init_guess_`
这些状态，**一个都没对外发布**，也没有配准残差输出。所以本节点只能给出
"位姿数据是否在正常更新"这一级别的判断，**发现不了"重定位收敛到了错误位姿"**。
相比之下 small_gicp/KISS 那两个节点能给到 converged/error 级别的判据。
这一限制记在 README 与 roadmap 里。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from super_lio_bridge import transforms as tf


class SuperLioBridge(Node):

    def __init__(self) -> None:
        super().__init__('super_lio_bridge')

        self.declare_parameter('lio_odom_topic', '/lio/odom')
        self.declare_parameter('lio_cloud_topic', '/lio/cloud_world')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Super-LIO 的 imu 帧 -> base_footprint。默认值来自 URDF 的
        # base_footprint ->(z+0.06) chassis ->(z+0.26) livox_frame，
        # 即雷达（≈MID-360 内置 IMU）在 base 上方 0.32m。
        # MID-360 内部 IMU 与雷达之间还有 2~4cm 的偏置，已经包含在 Super-LIO 自己的
        # 外参里，所以这里的残差约几厘米 —— 要更准就实测后改这个参数。
        self.declare_parameter('imu_to_base', [0.0, 0.0, -0.32, 0.0, 0.0, 0.0])
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('registered_scan_topic', '/registered_scan')
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('tracking_ok_topic', '/localization/tracking_ok')
        self.declare_parameter('tracking_status_topic', '/localization/tracking_status')
        # 位姿多久没更新就判为失效（进而锁住自动驾驶）
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        # 给 TF 时间戳加的补偿量，抵消 LIO 的处理延迟（与工程里既有的 +0.1s 约定一致）
        self.declare_parameter('stamp_offset', 0.1)
        # internal: Super-LIO 自己就是 map 系定位（relocation 模式），桥接层锚定 map->odom
        # external: Super-LIO 只当里程计（mapping 模式），map->odom 交给外部
        #           重定位节点（small_gicp_relocalization）持续发布
        self.declare_parameter('localization_mode', 'internal')

        self._map = self.get_parameter('map_frame').value
        self._odom = self.get_parameter('odom_frame').value
        self._base = self.get_parameter('base_frame').value
        self._timeout = float(self.get_parameter('odom_timeout').value)
        self._stamp_offset = float(self.get_parameter('stamp_offset').value)
        self._mode = str(self.get_parameter('localization_mode').value)
        if self._mode not in ('internal', 'external'):
            raise ValueError(
                "localization_mode 只能是 'internal' 或 'external'，"
                f'收到 {self._mode!r}')
        self._external = self._mode == 'external'
        offset = [float(v) for v in self.get_parameter('imu_to_base').value]
        if len(offset) != 6:
            raise ValueError('imu_to_base 需要 6 个数: x y z roll pitch yaw')
        self._T_imu_base = tf.make_T(
            tf.rpy_to_matrix(offset[3], offset[4], offset[5]), offset[:3])

        self._T_world_base: np.ndarray | None = None
        self._T_boot: np.ndarray | None = None
        self._last_stamp: float | None = None
        self._last_msg = None
        self._healthy: bool | None = None
        self._last_status = 0.0

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self._cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('registered_scan_topic').value, 10)

        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 外部重定位模式下，tracking_ok/status 由 small_gicp_relocalization 发布；
        # 本节点不能再发，否则同一个 topic 有两个发布者，看门狗会看到互相覆盖的判定。
        self._ok_pub = None
        self._status_pub = None
        if not self._external:
            self._ok_pub = self.create_publisher(
                Bool, self.get_parameter('tracking_ok_topic').value, latched)
            self._status_pub = self.create_publisher(
                String, self.get_parameter('tracking_status_topic').value, 10)

        self.create_subscription(
            Odometry, self.get_parameter('lio_odom_topic').value, self._on_odom, 10)
        if bool(self.get_parameter('publish_cloud').value):
            self.create_subscription(
                PointCloud2, self.get_parameter('lio_cloud_topic').value,
                self._on_cloud, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'Super-LIO 桥接已启动（localization_mode={self._mode}）：'
            f'{self.get_parameter("lio_odom_topic").value} -> '
            f'{self._map} -> {self._odom} -> {self._base}；'
            f'imu->base = {[round(v, 3) for v in offset]}'
        )

    # ------------------------------------------------------------------ 回调

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # 消息里的位姿是 world -> imu
        T_world_imu = tf.make_T(tf.quat_to_matrix(o.x, o.y, o.z, o.w),
                                [p.x, p.y, p.z])
        T_world_base = T_world_imu @ self._T_imu_base

        if self._T_boot is None:
            if self._external:
                # 外部重定位：Super-LIO 的 world 就是 odom（从机器人启动位姿开始），
                # 不再围绕第一帧做锚定。map->odom 由 small_gicp_relocalization 发布。
                self._T_boot = np.eye(4)
                self.get_logger().info(
                    '外部重定位模式：world 按 odom 使用，'
                    'map->odom 由 small_gicp_relocalization 发布；本节点不再发布该变换。')
            else:
                self._T_boot = T_world_base.copy()
                rpy = tf.rpy_of(self._T_boot)
                self.get_logger().info(
                    '已用第一帧定位结果锚定 map->odom：'
                    f'xyz=[{self._T_boot[0,3]:.3f}, {self._T_boot[1,3]:.3f}, '
                    f'{self._T_boot[2,3]:.3f}] rpy=[{rpy[0]:.3f}, {rpy[1]:.3f}, '
                    f'{rpy[2]:.3f}]。之后该变换保持不变（单坐标系 LIO 的固有特性）。')

        self._T_world_base = T_world_base
        self._last_stamp = time.monotonic()
        self._last_msg = msg

    def _on_cloud(self, msg: PointCloud2) -> None:
        # 转发给 /registered_scan，供 3D→2D 切片和重定位使用。
        #
        # 必须把 frame_id 从 Super-LIO 的 "world" 改成本工程 TF 树里存在的帧：
        #   internal -> map（点云已经是 map 系）
        #   external -> odom（Super-LIO 只做里程计，点云在 odom 系；外部重定位
        #               节点正是在 odom 系里把它配准到先验图）
        # 否则 pointcloud_to_laserscan（target_frame=livox_frame）转不过去，
        # 只会刷 "Message Filter dropping message ... queue is full"，/scan 出不来。
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self._odom if self._external else self._map
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self._cloud_pub.publish(out)

    # ------------------------------------------------------------------ 主循环

    def _on_timer(self) -> None:
        fresh = (self._last_stamp is not None and
                 time.monotonic() - self._last_stamp <= self._timeout)
        healthy = bool(fresh and self._T_boot is not None)

        if self._T_boot is None:
            if not self._external:
                # Keep the TF tree connected from t=0. Nav2 and explore_lite start
                # concurrently with this bridge; without a map->odom edge they fail
                # with 'Invalid frame ID "map"' until the first pose arrives.
                # This identity transform is replaced by the anchored one below.
                self._broadcast(self._map, self._odom, np.eye(4),
                                self.get_clock().now())
        elif healthy:
            if self._external:
                # world 就是 odom，不需要再绕第一帧做锚定。
                T_odom_base = self._T_world_base
            else:
                T_odom_base = tf.invert_T(self._T_boot) @ self._T_world_base
            stamp = self._stamp_of(self._last_msg)
            if not self._external:
                self._broadcast(self._map, self._odom, self._T_boot, stamp)
            self._broadcast(self._odom, self._base, T_odom_base, stamp)
            self._publish_odom(T_odom_base, stamp)

        # 外部模式下 tracking_ok/status 由 small_gicp_relocalization 发布。
        if not self._external:
            if healthy != self._healthy:
                self._healthy = healthy
                self._publish_status(fresh)
                if healthy:
                    self.get_logger().info('Super-LIO 位姿正常，已放行自动驾驶。')
                else:
                    self.get_logger().error(
                        'Super-LIO 位姿不可用（未初始化或超过 '
                        f'{self._timeout:.1f}s 没有更新），已锁住自动驾驶（遥控不受影响）。')
            elif healthy and time.monotonic() - self._last_status > 1.0:
                self._publish_status(fresh)

            self._ok_pub.publish(Bool(data=healthy))

    def _stamp_of(self, msg) -> rclpy.time.Time:
        t = rclpy.time.Time.from_msg(msg.header.stamp)
        return t + rclpy.duration.Duration(seconds=self._stamp_offset)

    def _broadcast(self, parent: str, child: str, T: np.ndarray,
                   stamp) -> None:
        q = tf.quat_of(T)
        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self._tf_broadcaster.sendTransform(msg)

    def _publish_odom(self, T_odom_base: np.ndarray, stamp) -> None:
        q = tf.quat_of(T_odom_base)
        out = Odometry()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self._odom
        out.child_frame_id = self._base
        out.pose.pose.position.x = float(T_odom_base[0, 3])
        out.pose.pose.position.y = float(T_odom_base[1, 3])
        out.pose.pose.position.z = float(T_odom_base[2, 3])
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])
        # Super-LIO 的 twist 是 imu 系的，本工程用的是 OPEN_LOOP 速度平滑，
        # 不需要它；这里不转发，避免给出一个坐标系没对齐的速度。
        self._odom_pub.publish(out)

    def _publish_status(self, fresh: bool) -> None:
        msg = String()
        if self._T_boot is None:
            state = 'waiting_for_first_pose'
        elif not fresh:
            state = 'pose_stale'
        else:
            state = 'tracking'
        msg.data = (f'state={state} tracking_ok={bool(self._healthy)} '
                    f'odom_timeout={self._timeout:.1f}s '
                    'note=Super-LIO 无对外残差输出，本判据仅反映位姿是否在更新')
        self._status_pub.publish(msg)
        self._last_status = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SuperLioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
