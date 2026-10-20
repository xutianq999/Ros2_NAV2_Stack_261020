"""把 Best Effort 的 /scan 中继成 Reliable 的 /scan_reliable。

为什么需要它
------------
pointcloud_to_laserscan 用 SensorDataQoS（Best Effort）发布 /scan，而 Nav2
costmap_2d 的 obstacle_layer 用 SystemDefaultsQoS（Reliable）订阅观测源。
两者 QoS 不兼容，启动日志里会出现：

  New subscription discovered on topic '/scan', requesting incompatible QoS.
  No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY

结果是 local/global costmap 的障碍层收不到任何扫描数据：MPPI 和
behavior_server 的碰撞检查都看不到墙，只剩 collision_monitor 在兜底。

本节点只做 QoS 中继：输入保持 Best Effort（与发布端一致），输出用 Reliable
供两份 costmap 订阅。collision_monitor 和 recovery_guard 继续直接订阅 /scan，
不经过这里，避免多一跳延迟。
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


class ScanQosRelay(Node):
    """Best Effort LaserScan -> Reliable LaserScan。"""

    def __init__(self) -> None:
        super().__init__('scan_qos_relay')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_reliable')
        self.declare_parameter('qos_depth', 5)

        self._input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        depth = max(1, int(self.get_parameter('qos_depth').value))

        reliable_qos = QoSProfile(
            depth=depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, reliable_qos)
        self.create_subscription(
            LaserScan, self._input_topic, self._on_scan,
            qos_profile_sensor_data)

        self._last_input = None
        self.create_timer(2.0, self._check_stale)

        self.get_logger().info(
            f'scan_qos_relay 启动：{self._input_topic} (best effort) -> '
            f'{output_topic} (reliable, depth={depth})')

    def _on_scan(self, msg: LaserScan) -> None:
        self._pub.publish(msg)
        self._last_input = self.get_clock().now()

    def _check_stale(self) -> None:
        if self._last_input is None:
            self.get_logger().warn(
                f'尚未收到 {self._input_topic}；costmap 会一直收不到扫描数据',
                throttle_duration_sec=10.0)
            return
        age = (self.get_clock().now() - self._last_input).nanoseconds * 1e-9
        if age > 2.0:
            self.get_logger().warn(
                f'{self._input_topic} 已 {age:.1f}s 没有更新',
                throttle_duration_sec=10.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanQosRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
