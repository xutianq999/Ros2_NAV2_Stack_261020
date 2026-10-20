"""定位/里程计健康看门狗。

自动导航全部建立在 TF 链健康的前提上：`odom -> base_footprint` 来自 LIO，
`map -> odom` 来自重定位节点。任何一环断掉，代价地图和规划结果都不再有意义。

本节点定期检查这两段 TF（以及可选的点云话题）是否新鲜，健康状态用一个 Bool
发布出去，交给 twist_mux 当"锁"用：

    true  = 锁住自动驾驶（不健康）
    false = 放行自动驾驶

关键设计取舍：看门狗**只掐自动驾驶，不掐遥控**。定位丢失时机器人最需要的是操作
员把它开回能重新定位的位置；如果连遥控一起锁死，现场就没有恢复手段了。硬急停是
另一个独立的高优先级输入（`/safety/estop`）。

twist_mux 的锁语义（已实测）：锁的优先级高于某话题时，该话题被停用。所以这里用
`blocked=true` 配一个介于遥控和导航之间的优先级，见 config/twist_mux.yaml。
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

BLOCKED = True
UNBLOCKED = False


def _declare_and_get(node: Node, name: str, default):
    """声明参数并取值，能容忍"空列表"这种无法推断类型的写法。

    YAML 里写 `pointcloud_checks: []` 时 ROS 2 推断不出数组元素类型，参数会处于
    NOT_SET 状态，直接 get_parameter 会抛 ParameterUninitializedException。
    用户想关掉某一类检查时很自然会这么写，所以这里不能崩，但也不能悄悄塞回默认值
    —— 对安全节点来说"静默恢复一项检查"和"静默丢掉一项检查"都不好。这里按显式
    空列表理解（该类检查关闭），并且明确告警。
    """
    node.declare_parameter(name, default)
    try:
        value = node.get_parameter(name).value
    except ParameterUninitializedException:
        if isinstance(default, list):
            node.get_logger().warn(f'参数 {name} 为空列表，该类检查将不执行。')
            return []
        return default
    return default if value is None else value


class Check:
    """一条健康检查项（TF / 点云话题 / 布尔状态）。"""

    def __init__(self, kind: str, name: str, timeout: float) -> None:
        self.kind = kind
        self.name = name
        self.timeout = timeout
        self.last_seen: float | None = None   # monotonic 时间戳，None = 从未观测到
        self.value: bool | None = None        # 仅 bool 检查项使用

    def age(self, now: float) -> float | None:
        if self.last_seen is None:
            return None
        return max(0.0, now - self.last_seen)

    def healthy(self, now: float) -> bool:
        age = self.age(now)
        if age is None or age > self.timeout:
            return False
        if self.kind == 'bool':
            # 布尔类检查必须显式为 true 才算健康；收到 false 也算不健康
            return self.value is True
        return True

    def describe(self, now: float) -> str:
        age = self.age(now)
        shown = 'n/a' if age is None else f'{age:.2f}s'
        if self.kind == 'bool':
            value = 'n/a' if self.value is None else str(self.value)
            shown = f'{value}, {shown}'
        return f'{self.name}({shown}/{self.timeout:.2f}s)'


class LocalizationWatchdog(Node):

    def __init__(self) -> None:
        super().__init__('localization_watchdog')

        # TF 检查项，格式 "<parent>-><child>"
        tf_names = list(_declare_and_get(
            self, 'tf_checks', ['map->odom', 'odom->base_footprint']))
        tf_timeouts = [float(t) for t in _declare_and_get(
            self, 'tf_timeouts', [1.0, 1.0])]
        # 点云话题检查项（PointCloud2）：至少要有消息进来
        cloud_names = list(_declare_and_get(
            self, 'pointcloud_checks', ['/registered_scan']))
        cloud_timeouts = [float(t) for t in _declare_and_get(
            self, 'pointcloud_timeouts', [1.0])]
        # 布尔状态检查项（Bool）：收到 false 或长时间没有消息都算不健康。
        # 默认盯重定位节点的跟踪健康度 —— 这是 TF 新鲜度发现不了的失效：
        # GICP 跟丢时不更新位姿，但 map->odom 还在按上一次的位姿继续发。
        bool_names = list(_declare_and_get(
            self, 'bool_checks', ['/localization/tracking_ok']))
        bool_timeouts = [float(t) for t in _declare_and_get(
            self, 'bool_timeouts', [2.0])]

        rate = max(1.0, float(_declare_and_get(self, 'publish_rate', 20.0)))
        # 启动宽限：刚起来时 LIO/重定位还没收敛，先按"锁住"处理，避免误放行
        self._grace = float(_declare_and_get(self, 'startup_grace', 5.0))
        self._blocked_topic = _declare_and_get(
            self, 'autonomy_blocked_topic', '/safety/autonomy_blocked')
        status_topic = _declare_and_get(
            self, 'status_topic', '/safety/watchdog_status')

        if len(tf_names) != len(tf_timeouts):
            raise ValueError('tf_checks 与 tf_timeouts 数量必须一致')
        if len(cloud_names) != len(cloud_timeouts):
            raise ValueError('pointcloud_checks 与 pointcloud_timeouts 数量必须一致')
        if len(bool_names) != len(bool_timeouts):
            raise ValueError('bool_checks 与 bool_timeouts 数量必须一致')

        self._tf_checks: list[Check] = []
        for name, timeout in zip(tf_names, tf_timeouts):
            if '->' not in name:
                raise ValueError(f'TF 检查项 "{name}" 格式应为 "parent->child"')
            parent, child = (part.strip() for part in name.split('->', 1))
            check = Check('tf', name, timeout)
            check.parent = parent          # type: ignore[attr-defined]
            check.child = child            # type: ignore[attr-defined]
            self._tf_checks.append(check)

        self._cloud_checks: list[Check] = []
        for name, timeout in zip(cloud_names, cloud_timeouts):
            check = Check('pointcloud', name, timeout)
            self._cloud_checks.append(check)

        for check in self._cloud_checks:
            self.create_subscription(
                PointCloud2, check.name, self._make_cloud_callback(check), 10)

        self._bool_checks: list[Check] = []
        for name, timeout in zip(bool_names, bool_timeouts):
            check = Check('bool', name, timeout)
            self._bool_checks.append(check)

        for check in self._bool_checks:
            self.create_subscription(
                Bool, check.name, self._make_bool_callback(check), 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # transient_local：twist_mux 后启动也能读到当前状态，不会因为错过一条
        # 消息就误判成"安全"
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._blocked_pub = self.create_publisher(Bool, self._blocked_topic, latched)
        self._status_pub = self.create_publisher(String, status_topic, 10)

        self._start = time.monotonic()
        self._blocked: bool | None = None
        self._last_status_pub = 0.0
        self.create_timer(1.0 / rate, self._on_timer)

        # 聚合进 /diagnostics，方便以后接监控
        self._updater = Updater(self)
        self._updater.setHardwareID('nav2_safety/localization_watchdog')
        self._updater.add('localization/odometry health', self._diag_task)

        self.get_logger().info(
            '定位看门狗启动：检查 '
            f'{[c.name for c in self._tf_checks + self._cloud_checks + self._bool_checks]}，'
            f'启动宽限 {self._grace:.1f}s，输出 {self._blocked_topic}'
        )

    # ------------------------------------------------------------- 回调

    def _make_cloud_callback(self, check: Check):
        def _callback(_msg: PointCloud2) -> None:
            check.last_seen = time.monotonic()

        return _callback

    def _make_bool_callback(self, check: Check):
        def _callback(msg: Bool) -> None:
            check.last_seen = time.monotonic()
            check.value = bool(msg.data)

        return _callback

    def _probe_tf(self) -> None:
        """用 TF 自身的时间戳计算年龄，避免时钟域不一致带来的误判。"""
        now = self.get_clock().now()
        for check in self._tf_checks:
            try:
                tf = self._tf_buffer.lookup_transform(
                    check.parent, check.child, rclpy.time.Time())
            except Exception:
                check.last_seen = None
                continue
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            age = (now - stamp).nanoseconds / 1e9
            check.last_seen = time.monotonic() - max(0.0, age)

    # ------------------------------------------------------------- 主循环

    def _on_timer(self) -> None:
        self._probe_tf()
        now = time.monotonic()
        checks = self._tf_checks + self._cloud_checks + self._bool_checks

        if now - self._start < self._grace:
            blocked = BLOCKED
            in_grace = True
        else:
            blocked = any(not c.healthy(now) for c in checks)
            in_grace = False

        if blocked != self._blocked:
            self._blocked = blocked
            detail = ', '.join(c.describe(now) for c in checks)
            if blocked and in_grace:
                # 宽限期内按锁住处理是预期行为，不该报成"不健康"——否则排障时
                # 会看到括号里各项明明都健康、却在报故障的错误日志。
                self.get_logger().info(
                    f'启动宽限期内，暂按锁住自动驾驶处理（遥控不受影响）：{detail}')
            elif blocked:
                self.get_logger().error(
                    f'定位/里程计不健康，已锁住自动驾驶（遥控不受影响）：{detail}')
            else:
                self.get_logger().info(f'定位/里程计恢复健康，放行自动驾驶：{detail}')
            self._publish_status(checks, now)
        elif now - self._last_status_pub > 1.0:
            # 低频重发：网页等易失订阅者（volatile）也能稳定拿到当前状态
            self._publish_status(checks, now)

        self._blocked_pub.publish(Bool(data=blocked))
        self._diag_checks = checks
        self._updater.update()

    def _diag_task(self, stat):
        checks = getattr(self, '_diag_checks', [])
        now = time.monotonic()
        if self._blocked:
            stat.summary(DiagnosticStatus.WARN, '定位/里程计不健康，已锁住自动驾驶')
        else:
            stat.summary(DiagnosticStatus.OK, '定位/里程计健康')
        for c in checks:
            age = c.age(now)
            stat.add(c.name, 'never' if age is None else f'{age:.2f}')
        return stat

    def _publish_status(self, checks, now: float) -> None:
        msg = String()
        msg.data = (f'autonomy_blocked={bool(self._blocked)} | '
                    + ', '.join(c.describe(now) for c in checks))
        self._status_pub.publish(msg)
        self._last_status_pub = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationWatchdog()
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
