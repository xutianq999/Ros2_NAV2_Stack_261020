"""软件急停锁存。

为什么需要单独一个节点：

急停的语义必须"设置一次、持续有效"，但话题本身不保证这一点。原先的做法是网页
直接往 /safety/estop 发一条 true，由 twist_mux 当锁用（timeout=0 表示不过期）。
这有两个问题：

1. rosbridge 的发布是易失的（volatile），没有锁存语义。仲裁层一旦重启，就再也
   收不到那条 true，锁没了，机器人可以动了 —— 而操作员在界面上看到的可能还是
   "已锁存"。
2. 网页重连后订阅 /safety/estop 也拿不到当前值，只能显示"未触发"，即使急停其实
   还锁着。

本节点把锁存状态从"一条一次性消息"变成"持续发布的状态"：
    输入 /safety/estop (Bool，写一次即可)
    输出 /safety/estop_active (Bool，transient_local，按 publish_rate 持续发布)

仲裁层锁 /safety/estop_active，界面也读它。这样后加入的订阅者、以及重启后的
仲裁层，都能立刻拿到当前锁存状态。

注意：这是**软件急停**，和底盘硬件急停按钮是两回事，两者都应保留。
"""

from __future__ import annotations

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class EstopLatch(Node):
    """把一次性急停指令锁存成可持久读取的状态。"""

    def __init__(self) -> None:
        super().__init__('estop_latch')

        self.declare_parameter('input_topic', '/safety/estop')
        self.declare_parameter('output_topic', '/safety/estop_active')
        self.declare_parameter('status_topic', '/safety/estop_status')
        self.declare_parameter('publish_rate', 2.0)
        # 非空时把锁存状态落盘：节点被 respawn 后不会静默解除急停。
        self.declare_parameter('state_file', '')

        rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self._state_file = str(self.get_parameter('state_file').value)
        self._latched = self._load_state()

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(
            Bool, self.get_parameter('output_topic').value, latched_qos)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        # 输入必须用**默认（volatile）**QoS：ROS 2 要求发布端持久性 >= 订阅端。
        # 如果这里用 transient_local，就收不到 volatile 发布者（网页按钮、
        # ros2 topic pub）的消息 —— QoS 不兼容，且只在日志里出现一行警告。
        self.create_subscription(
            Bool, self.get_parameter('input_topic').value, self._on_input,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'急停锁存已启动：{self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value} '
            f'(transient_local, {rate:.1f}Hz)；当前='
            f'{"已触发（从 state_file 恢复）" if self._latched else "未触发"}'
        )
        # 立即发布一次，别让 twist_mux 在第一个定时器周期前处于"从未收到锁"状态。
        self._publish()

    def _load_state(self) -> bool:
        if not self._state_file:
            return False
        try:
            with open(self._state_file, 'r', encoding='utf-8') as f:
                return f.read().strip().lower() in ('1', 'true', 'yes')
        except FileNotFoundError:
            return False
        except OSError as exc:
            self.get_logger().warn(
                f'读取急停状态文件 {self._state_file} 失败: {exc}；按未触发处理')
            return False

    def _save_state(self) -> None:
        if not self._state_file:
            return
        tmp = f'{self._state_file}.tmp'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                f.write('true' if self._latched else 'false')
            os.replace(tmp, self._state_file)
        except OSError as exc:
            self.get_logger().error(
                f'写入急停状态文件 {self._state_file} 失败: {exc}')

    def _on_input(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested == self._latched:
            return
        self._latched = requested
        if requested:
            self.get_logger().error(
                '急停已锁存。仲裁层会压制所有速度来源（含遥控）；'
                f'排查后向 {self.get_parameter("input_topic").value} 发 false 解除。'
            )
        else:
            self.get_logger().warn('急停已解除，恢复按优先级放行速度指令。')
        self._save_state()
        self._publish()

    def _on_timer(self) -> None:
        # 持续发布：后加入的订阅者、以及重启后的仲裁层都能拿到当前状态
        self._publish()

    def _publish(self) -> None:
        self._pub.publish(Bool(data=self._latched))
        status = String()
        status.data = f'estop_latched={self._latched}'
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstopLatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
