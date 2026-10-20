"""速度指令兜底归零（deadman / zero-injector）。

为什么需要这个节点 —— 老实说，它的价值比最初设想的低：

**BUNKER 用户手册 Q&A 明确写着**：底盘在处理来自外部的 CAN 控制指令时有超时保护，
超过 500ms 没收到下一帧控制指令就进入通讯保护、速度为 0。所以"发布者挂掉导致车
继续跑"这个场景**底盘本来就会自己处理**。我最初声称"底盘会保留最后一条运动指令
继续行驶"是错的 —— 当时只核了 ROS 节点（`bunker_messenger.hpp` 确实只在收到
`/cmd_vel` 时才调 `SetMotionCommand`，没有定时重发、没有超时保护），却没去查固件
就把它当成了结论。

本节点仍然成立的一点：`twist_mux` 在输入全部超时或被锁挡住时是**静默**的，而
"静默"要靠固件那 500ms 超时来兜底；本节点把静默换成显式零速，让停车行为不依赖
固件的超时实现，并把延迟从 500ms 缩到 300ms（0.26 m/s 下约 5cm）。

结论：这是**可选**的锦上添花，不是安全底线。真正补上缺口的是 twist_mux（优先级
仲裁）和 collision_monitor（独立障碍闸）。不想多一个节点可以不起它。

插在仲裁层和底盘之间：
    twist_mux -> cmd_vel_muxed -> [本节点] -> /cmd_vel -> 底盘

输入新鲜时按原样转发（不改变时序），一旦超过 timeout 没收到输入，就以
publish_rate 持续下发零速，直到输入恢复。

输出话题故意选 `/cmd_vel`：底盘驱动把该话题硬编码在 C++ 里（bunker_messenger.hpp），
不想为接安全层去改底盘代码。代价是启用本层之后，`/cmd_vel` 必须由本节点独占，
所以这里带一个自检：一旦发现 /cmd_vel 上还有别的发布者（例如没改话题的遥控、
或旧版 Nav2 启动文件），立刻报错并说清该怎么改 —— 否则两路指令会在底盘上交错。
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Deadman(Node):
    """把上游的静默转成显式零速。"""

    def __init__(self) -> None:
        super().__init__('cmd_vel_deadman')

        self.declare_parameter('input_topic', 'cmd_vel_muxed')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('timeout', 0.3)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('status_topic', '/safety/deadman_status')
        # 给网页等易失订阅者的布尔状态：true = 正在下发零速
        self.declare_parameter('stopping_topic', '/safety/deadman_stopping')
        # 检查输出话题上有没有本节点之外的发布者
        self.declare_parameter('foreign_publisher_check_period', 2.0)

        self._timeout = float(self.get_parameter('timeout').value)
        self._output_topic = self.get_parameter('output_topic').value
        rate = max(1.0, float(self.get_parameter('publish_rate').value))

        if self._timeout <= 0.0:
            raise ValueError('timeout 必须为正数，否则无法区分"上游静默"和"指令正常"')

        self._last_stamp: float | None = None
        self._stopping = False
        self._warned_foreign = False

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self._stopping_pub = self.create_publisher(
            Bool, self.get_parameter('stopping_topic').value, 10)
        self._last_stopping_pub = 0.0
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._on_cmd, 10)
        self.create_timer(1.0 / rate, self._on_timer)
        self.create_timer(
            max(0.5, float(self.get_parameter('foreign_publisher_check_period').value)),
            self._check_foreign_publishers)

        # 聚合进 /diagnostics，方便以后接监控
        self._updater = Updater(self)
        self._updater.setHardwareID('nav2_safety/cmd_vel_deadman')
        self._updater.add('cmd_vel deadman', self._diag_task)

        self.get_logger().info(
            f'兜底归零已启用：{self.get_parameter("input_topic").value} -> '
            f'{self._output_topic}，'
            f'静默超过 {self._timeout:.2f}s 即持续下发零速'
        )

    def _diag_task(self, stat):
        if self._stopping:
            stat.summary(DiagnosticStatus.WARN, '上游静默，正在持续下发零速')
        else:
            stat.summary(DiagnosticStatus.OK, '透传上游速度指令')
        stat.add('timeout_s', f'{self._timeout:.2f}')
        stat.add('input_topic', str(self.get_parameter('input_topic').value))
        stat.add('output_topic', str(self._output_topic))
        return stat

    def _check_foreign_publishers(self) -> None:
        """输出话题必须由本节点独占，否则停车指令会和别路指令在底盘上交错。"""
        try:
            infos = self.get_publishers_info_by_topic(self._output_topic)
        except Exception:
            return
        others = sorted({info.node_name for info in infos
                         if info.node_name != self.get_name()})
        if others:
            if not self._warned_foreign:
                self._warned_foreign = True
                self.get_logger().error(
                    f'话题 {self._output_topic} 上除本节点外还有发布者：{others}。'
                    '两路速度指令会在底盘上交错，可能出现抖动或停车失效。'
                    '若那是遥控发布者，请改到 /cmd_vel_teleop；'
                    '若那是旧版 Nav2 启动文件，请改用 nav2_safety 的安全版启动。'
                    '详见 nav2_safety/docs/ENABLING.md。'
                )
        elif self._warned_foreign:
            self._warned_foreign = False
            self.get_logger().info(f'{self._output_topic} 已由本节点独占。')

    def _on_cmd(self, msg: Twist) -> None:
        self._last_stamp = time.monotonic()
        if self._stopping:
            self._stopping = False
            self.get_logger().info('上游速度指令恢复，解除兜底停车。')
            self._publish_status()
        # 原样转发，保持上游时序不被本节点改变
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        if self._last_stamp is None:
            # 从未收到过上游指令：初始状态即视为静默
            stale = True
        else:
            stale = (time.monotonic() - self._last_stamp) > self._timeout

        if stale:
            if not self._stopping:
                self._stopping = True
                self.get_logger().warn(
                    f'上游静默超过 {self._timeout:.2f}s，持续下发零速（底盘将停车）。'
                )
                self._publish_status()
            self._pub.publish(Twist())

        # 状态低频重发，让易失订阅者（网页）也能稳定拿到
        now = time.monotonic()
        if now - self._last_stopping_pub > 0.5:
            self._stopping_pub.publish(Bool(data=self._stopping))
            self._last_stopping_pub = now
        self._updater.update()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = (
            f'stopping={self._stopping} timeout={self._timeout:.2f} '
            f'input={self.get_parameter("input_topic").value}'
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Deadman()
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
