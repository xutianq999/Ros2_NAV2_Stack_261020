"""机器人实时状态聚合（只读）。

把 Nav2、行为树恢复动作、安全层、定位、探索的状态汇总成一个稳定的 JSON
状态，发布到 /robot/status，供 Web 导航页和 RViz 显示。

状态优先级（从高到低）：
  ESTOP       软件急停锁存
  STARTING    启动宽限期内（看门狗暂时锁住自动驾驶）
  SAFETY_LOCK 安全层锁住自动驾驶（定位/TF/传感器异常）
  RECOVERING  恢复行为执行中（脱困）
  STUCK       导航中、有速度指令、但一段时间没有进展
  NAVIGATING  导航 action 执行中
  EXPLORING   前沿探索进行中
  EXPLORE_DONE 探索完成
  IDLE        空闲

本节点不参与任何控制或安全决策，只做观测。
"""

from __future__ import annotations

import json
import math

import rclpy
from action_msgs.msg import GoalStatusArray
from bunker_msgs.msg import BunkerStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Twist
# Nav2 的 action __init__ 只导出 NavigateToPose 本体，FeedbackMessage 在私有
# 生成模块里；rclpy 订阅 /_action/feedback 需要这个包装类型。
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
ACTIVE_STATUSES = (STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING)

RECOVERY_ACTIONS = (
    'smart_recovery',
    'backup',
    'spin',
    'wait',
    'drive_on_heading',
)
RECOVERY_LABELS = {
    'smart_recovery': '智能脱困',
    'backup': '倒车',
    'spin': '原地旋转',
    'wait': '等待',
    'drive_on_heading': '沿朝向直行',
    'recovery': '恢复动作',
}

STATE_LABELS = {
    'IDLE': '空闲',
    'STARTING': '启动中',
    'ESTOP': '急停',
    'SAFETY_LOCK': '安全层锁定',
    'RECOVERING': '脱困中',
    'STUCK': '卡住',
    'NAVIGATING': '导航中',
    'EXPLORING': '探索中',
    'EXPLORE_DONE': '探索完成',
}

# RViz 的默认字体对中文支持不好，marker 用英文状态名。
MARKER_COLORS = {
    'IDLE': (0.60, 0.60, 0.60),
    'STARTING': (0.60, 0.60, 0.60),
    'NAVIGATING': (0.29, 0.87, 0.50),
    'EXPLORING': (0.13, 0.83, 0.93),
    'RECOVERING': (0.23, 0.51, 0.96),
    'STUCK': (0.98, 0.57, 0.24),
    'ESTOP': (0.94, 0.27, 0.27),
    'SAFETY_LOCK': (0.94, 0.27, 0.27),
    'EXPLORE_DONE': (0.29, 0.87, 0.50),
}


class RobotStatusMonitor(Node):
    """只读聚合节点：/robot/status (JSON String) + /robot/status_markers。"""

    def __init__(self) -> None:
        super().__init__('robot_status_monitor')

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('stuck_timeout', 12.0)
        self.declare_parameter('progress_epsilon', 0.05)
        self.declare_parameter('yaw_progress_epsilon', 0.2)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('nav_status_timeout', 2.5)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('startup_grace', 8.0)
        self.declare_parameter('cmd_linear_threshold', 0.02)
        self.declare_parameter('cmd_angular_threshold', 0.1)
        self.declare_parameter('collision_stop_time', 0.5)
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_frame', 'base_footprint')
        self.declare_parameter('marker_height', 1.1)
        self.declare_parameter('battery_topic', '/bunker_status')
        self.declare_parameter('battery_timeout', 2.0)
        self.declare_parameter('battery_low_soc', 20.0)
        self.declare_parameter('battery_critical_soc', 10.0)
        self.declare_parameter('battery_voltage_full', 29.4)
        self.declare_parameter('battery_voltage_empty', 21.0)
        self.declare_parameter('battery_charge_current_sign', -1.0)
        self.declare_parameter('battery_charge_current_threshold', 0.2)

        gp = self.get_parameter
        self._stuck_timeout = float(gp('stuck_timeout').value)
        self._progress_epsilon = float(gp('progress_epsilon').value)
        self._yaw_progress_epsilon = float(gp('yaw_progress_epsilon').value)
        self._cmd_timeout = float(gp('cmd_timeout').value)
        self._nav_status_timeout = float(gp('nav_status_timeout').value)
        self._odom_timeout = float(gp('odom_timeout').value)
        self._scan_timeout = float(gp('scan_timeout').value)
        self._startup_grace = float(gp('startup_grace').value)
        self._cmd_lin_threshold = float(gp('cmd_linear_threshold').value)
        self._cmd_ang_threshold = float(gp('cmd_angular_threshold').value)
        self._collision_stop_time = float(gp('collision_stop_time').value)
        self._publish_marker = bool(gp('publish_marker').value)
        self._marker_frame = str(gp('marker_frame').value)
        self._marker_height = float(gp('marker_height').value)
        self._battery_topic = str(gp('battery_topic').value)
        self._battery_timeout = float(gp('battery_timeout').value)
        self._battery_low_soc = float(gp('battery_low_soc').value)
        self._battery_critical_soc = float(gp('battery_critical_soc').value)
        self._battery_voltage_full = float(gp('battery_voltage_full').value)
        self._battery_voltage_empty = float(gp('battery_voltage_empty').value)
        self._battery_charge_current_sign = float(
            gp('battery_charge_current_sign').value)
        self._battery_charge_current_threshold = float(
            gp('battery_charge_current_threshold').value)

        now = self.get_clock().now()
        self._start_time = now

        # 安全层 / 定位
        self._estop = False
        self._autonomy_blocked = False
        self._tracking_ok = None
        self._deadman_stopping = False
        self._recovery_blocked = False
        self._recovery_guard_active = False
        self._recovery_guard_active_time = None
        self._watchdog_detail = ''
        self._recovery_guard_detail = ''
        self._recovery_guard_time = None

        # 导航
        self._nav_active = False
        self._nav_status = None
        self._nav_status_time = None
        self._nav_goal_id = None
        self._distance_remaining = None
        self._number_of_recoveries = 0
        self._last_recoveries = 0
        self._last_goal_distance = None

        # 运动与进展
        self._last_progress = now
        self._cmd_active_since = now
        self._cmd_was_active = False
        self._cmd_smoothed_active = False
        self._cmd_smoothed_time = None
        self._checked_zero_since = None
        self._last_odom_pose = None
        self._odom_time = None
        self._scan_time = None
        self._recovery_cmd_time = None

        # 恢复动作
        self._recovery_active = {a: False for a in RECOVERY_ACTIONS}
        self._recovery_status_time = {a: None for a in RECOVERY_ACTIONS}

        # 探索
        self._explore_status = None

        # 电池 / BMS
        self._battery_msg = None
        self._battery_time = None

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_pub = self.create_publisher(String, '/robot/status', latched)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/robot/status_markers', 10)

        self.create_subscription(Bool, '/safety/estop_active', self._on_estop, latched)
        self.create_subscription(
            Bool, '/safety/autonomy_blocked', self._on_autonomy, latched)
        self.create_subscription(
            Bool, '/localization/tracking_ok', self._on_tracking, latched)
        self.create_subscription(
            Bool, '/safety/deadman_stopping', self._on_deadman, 10)
        self.create_subscription(
            Bool, '/safety/recovery_blocked', self._on_recovery_blocked, 10)
        self.create_subscription(
            Bool, '/safety/recovery_active', self._on_recovery_active, 10)
        self.create_subscription(
            String, '/safety/recovery_status', self._on_recovery_guard_status, 10)
        self.create_subscription(
            String, '/safety/watchdog_status', self._on_watchdog, 10)
        self.create_subscription(
            ExploreStatus, '/explore/status', self._on_explore, latched)

        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, latched)
        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback', self._on_feedback, 10)

        for action in RECOVERY_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                self._make_recovery_cb(action), latched)

        self.create_subscription(
            Twist, '/cmd_vel_nav_smoothed', self._on_cmd_smoothed, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav_checked', self._on_cmd_checked, 10)
        self.create_subscription(
            Twist, '/cmd_vel_recovery', self._on_cmd_recovery, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            BunkerStatus, self._battery_topic, self._on_battery, 10)

        rate = max(0.5, float(gp('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'robot_status_monitor 启动：/robot/status (JSON, %.1fHz) + '
            '/robot/status_markers（marker=%s）' % (rate, self._publish_marker))

    # ------------------------------------------------------------------ utils
    def _now(self):
        return self.get_clock().now()

    def _age(self, stamp) -> float | None:
        if stamp is None:
            return None
        return (self._now() - stamp).nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    def _on_autonomy(self, msg: Bool) -> None:
        self._autonomy_blocked = bool(msg.data)

    def _on_tracking(self, msg: Bool) -> None:
        self._tracking_ok = bool(msg.data)

    def _on_deadman(self, msg: Bool) -> None:
        self._deadman_stopping = bool(msg.data)

    def _on_recovery_blocked(self, msg: Bool) -> None:
        self._recovery_blocked = bool(msg.data)

    def _on_recovery_active(self, msg: Bool) -> None:
        # 20 Hz heartbeat from recovery_guard; unlike the latched action status
        # it stays fresh for the whole action, however long it runs.
        self._recovery_guard_active = bool(msg.data)
        self._recovery_guard_active_time = self._now()

    def _on_watchdog(self, msg: String) -> None:
        self._watchdog_detail = str(msg.data)

    def _on_explore(self, msg: ExploreStatus) -> None:
        # ExploreStatus is a latched transition event, not a heartbeat. Keep it
        # until the next transition (pause, completion, blocked or resume).
        self._explore_status = str(msg.status)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        now = self._now()
        self._nav_status_time = now
        active = [s for s in msg.status_list if s.status in ACTIVE_STATUSES]
        if active:
            latest = max(
                active,
                key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
            if goal_id != self._nav_goal_id:
                self._nav_goal_id = goal_id
                self._last_progress = now
                self._last_goal_distance = None
                self._last_recoveries = 0
            self._nav_active = True
            self._nav_status = int(latest.status)
        else:
            self._nav_active = False
            self._nav_goal_id = None
            self._last_goal_distance = None
            if msg.status_list:
                latest = max(
                    msg.status_list,
                    key=lambda s: (s.goal_info.stamp.sec,
                                   s.goal_info.stamp.nanosec))
                self._nav_status = int(latest.status)
            else:
                self._nav_status = None

    def _on_feedback(self, msg: NavigateToPose_FeedbackMessage) -> None:
        # Action status is emitted on state changes. Fresh feedback keeps the
        # current goal live even when EXECUTING has not changed for minutes.
        # Late feedback from a replaced/finished goal must not revive it.
        goal_id = bytes(msg.goal_id.uuid).hex()
        if not self._nav_active or goal_id != self._nav_goal_id:
            return
        now = self._now()
        self._nav_status_time = now
        fb = msg.feedback
        self._number_of_recoveries = int(fb.number_of_recoveries)
        distance = float(fb.distance_remaining)
        self._distance_remaining = distance

        if self._number_of_recoveries != self._last_recoveries:
            self._last_recoveries = self._number_of_recoveries
            self._last_progress = now

        if self._last_goal_distance is None:
            self._last_goal_distance = distance
        elif distance < self._last_goal_distance - self._progress_epsilon:
            self._last_goal_distance = distance
            self._last_progress = now

    def _make_recovery_cb(self, action):
        def _cb(msg: GoalStatusArray) -> None:
            self._recovery_active[action] = any(
                s.status in ACTIVE_STATUSES for s in msg.status_list)
            self._recovery_status_time[action] = self._now()
        return _cb

    def _on_cmd_smoothed(self, msg: Twist) -> None:
        now = self._now()
        self._cmd_smoothed_time = now
        active = (
            abs(msg.linear.x) > self._cmd_lin_threshold or
            abs(msg.angular.z) > self._cmd_ang_threshold)
        if active and not self._cmd_was_active:
            self._cmd_active_since = now
        self._cmd_was_active = active
        self._cmd_smoothed_active = active

    def _on_cmd_checked(self, msg: Twist) -> None:
        zero = (
            abs(msg.linear.x) < 0.005 and
            abs(msg.angular.z) < 0.005)
        if zero:
            if self._checked_zero_since is None:
                self._checked_zero_since = self._now()
        else:
            self._checked_zero_since = None

    def _on_cmd_recovery(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.005 or abs(msg.angular.z) > 0.005:
            self._recovery_cmd_time = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        now = self._now()
        self._odom_time = now
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (p.position.x, p.position.y, yaw)
        if self._last_odom_pose is None:
            self._last_odom_pose = pose
            return
        px, py, pyaw = self._last_odom_pose
        dist = math.hypot(pose[0] - px, pose[1] - py)
        dyaw = abs(math.atan2(
            math.sin(pose[2] - pyaw), math.cos(pose[2] - pyaw)))
        if dist > self._progress_epsilon or dyaw > self._yaw_progress_epsilon:
            self._last_progress = now
            self._last_odom_pose = pose

    def _on_scan(self, _msg: LaserScan) -> None:
        self._scan_time = self._now()

    def _on_battery(self, msg: BunkerStatus) -> None:
        self._battery_msg = msg
        self._battery_time = self._now()

    def _battery_payload(self, now) -> dict:
        """Battery snapshot for /robot/status.

        Prefer the BMS SOC. Some BUNKER firmware does not send BMS frames;
        in that case fall back to a linear voltage estimate using the
        configurable full/empty voltages. Unknown values are None (not 0).
        """
        msg = self._battery_msg
        age = self._age(self._battery_time)
        if msg is None or age is None or age > self._battery_timeout:
            return {
                'online': False,
                'age': None if age is None else round(age, 2),
                'voltage': None, 'soc': None, 'soh': None,
                'current': None, 'temperature': None,
                'charging': None, 'source': 'none',
                'low': False, 'critical': False,
            }

        voltage = float(msg.battery_voltage)
        soc = int(msg.battery_soc)
        soh = int(msg.battery_soh)
        current = float(msg.battery_current)
        temperature = float(msg.battery_temperature)
        source = 'bms'
        if soc <= 0 or soc > 100:
            soc = None
            source = 'voltage'
            span = self._battery_voltage_full - self._battery_voltage_empty
            if voltage > 0.0 and span > 0.0:
                pct = 100.0 * (voltage - self._battery_voltage_empty) / span
                soc = int(round(max(0.0, min(100.0, pct))))
        charging = (
            current * self._battery_charge_current_sign >=
            self._battery_charge_current_threshold)
        low = soc is not None and soc <= self._battery_low_soc
        critical = soc is not None and soc <= self._battery_critical_soc
        return {
            'online': True,
            'age': round(age, 2),
            'voltage': round(voltage, 2),
            'soc': soc,
            'soh': soh if 0 <= soh <= 100 else None,
            'current': round(current, 2),
            'temperature': round(temperature, 1),
            'charging': bool(charging),
            'source': source,
            'low': bool(low),
            'critical': bool(critical),
        }

    # ------------------------------------------------------------------ state
    def _active_recovery(self, now):
        # The guard's heartbeat is authoritative while it is alive: the action
        # status topics are latched and may not change for the entire 20 s
        # recovery, so an age check on them alone would drop RECOVERING after
        # nav_status_timeout seconds.
        heartbeat_age = self._age(self._recovery_guard_active_time)
        if (self._recovery_guard_active and heartbeat_age is not None and
                heartbeat_age <= 1.0):
            for action in RECOVERY_ACTIONS:
                if self._recovery_active.get(action):
                    return True, action
            return True, 'recovery'
        for action in RECOVERY_ACTIONS:
            if not self._recovery_active.get(action):
                continue
            age = self._age(self._recovery_status_time.get(action))
            if age is not None and age <= self._nav_status_timeout:
                return True, action
        cmd_age = self._age(self._recovery_cmd_time)
        if cmd_age is not None and cmd_age <= self._cmd_timeout:
            return True, 'recovery'
        return False, None

    def _tick(self) -> None:
        now = self._now()

        nav_age = self._age(self._nav_status_time)
        nav_active = (
            self._nav_active and nav_age is not None and
            nav_age <= self._nav_status_timeout)
        recovery_active, recovery_action = self._active_recovery(now)

        cmd_age = self._age(self._cmd_smoothed_time)
        cmd_active = (
            self._cmd_smoothed_active and cmd_age is not None and
            cmd_age <= self._cmd_timeout)

        if recovery_active:
            # 恢复期间刷新进展计时器，避免刚脱困结束就误报"卡住"。
            self._last_progress = now

        stuck_for = 0.0
        if nav_active and cmd_active and not recovery_active:
            ref = max(self._last_progress, self._cmd_active_since)
            stuck_for = max(0.0, (now - ref).nanoseconds * 1e-9)
        stuck = stuck_for >= self._stuck_timeout

        collision_stop = False
        if cmd_active:
            zero_age = self._age(self._checked_zero_since)
            collision_stop = (
                zero_age is not None and zero_age >= self._collision_stop_time)
        else:
            # 没有速度指令时，checked 为零是正常状态，不能把计时器带进下一段导航。
            self._checked_zero_since = None

        odom_age = self._age(self._odom_time)
        scan_age = self._age(self._scan_time)
        odom_ok = odom_age is not None and odom_age <= self._odom_timeout
        scan_ok = scan_age is not None and scan_age <= self._scan_timeout
        ages = [a for a in (odom_age, nav_age) if a is not None]
        data_age = max(ages) if ages else 999.0

        state, detail = self._choose_state(
            nav_active, recovery_active, recovery_action, stuck,
            stuck_for, collision_stop)
        label = STATE_LABELS.get(state, state)

        payload = {
            'stamp': now.nanoseconds * 1e-9,
            'state': state,
            'label': label,
            'detail': detail,
            'nav_active': bool(nav_active),
            'nav_status': self._nav_status,
            'recovery_active': bool(recovery_active),
            'recovery_action': recovery_action or '',
            'recovery_blocked': bool(self._recovery_blocked),
            'stuck': bool(stuck),
            'stuck_for': round(stuck_for, 1),
            'collision_stop': bool(collision_stop),
            'estop': bool(self._estop),
            'autonomy_blocked': bool(self._autonomy_blocked),
            'tracking_ok': self._tracking_ok,
            'odom_ok': bool(odom_ok),
            'scan_ok': bool(scan_ok),
            'deadman_stopping': bool(self._deadman_stopping),
            'explore_status': self._explore_status,
            'distance_remaining': (
                None if self._distance_remaining is None
                else round(float(self._distance_remaining), 3)),
            'number_of_recoveries': int(self._number_of_recoveries),
            'data_age': round(min(data_age, 999.0), 2),
            'battery': self._battery_payload(now),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

        if self._publish_marker:
            self._publish_marker_msg(state, recovery_action, stuck_for)

    def _on_recovery_guard_status(self, msg):
        try:
            payload = json.loads(msg.data)
            self._recovery_guard_detail = str(payload.get('detail', ''))
            self._recovery_guard_time = self._now()
        except (ValueError, TypeError, AttributeError):
            return

    def _choose_state(
            self, nav_active, recovery_active, recovery_action,
            stuck, stuck_for, collision_stop):
        if self._estop:
            return 'ESTOP', '软件急停已锁存；发 false 解除'

        if self._autonomy_blocked:
            age = self._age(self._start_time)
            if age is not None and age <= self._startup_grace:
                return 'STARTING', '等待定位/传感器就绪（看门狗启动宽限）'
            detail = self._watchdog_detail or '自动驾驶被安全层锁住'
            return 'SAFETY_LOCK', detail

        if recovery_active:
            label = RECOVERY_LABELS.get(recovery_action, recovery_action)
            guard_age = self._age(self._recovery_guard_time)
            if guard_age is not None and guard_age < 2.0 and self._recovery_guard_detail:
                return 'RECOVERING', f'{label}：{self._recovery_guard_detail}'
            if self._recovery_blocked:
                return 'RECOVERING', f'{label}：朝障碍方向的分量被拦截'
            return 'RECOVERING', label

        if self._explore_status == 'exploration_blocked':
            return 'STUCK', '探索暂停：导航失败且小车未有效平移，请检查起点代价图/定位后恢复探索'

        if stuck:
            detail = f'已有 {stuck_for:.0f}s 没有进展'
            if collision_stop:
                detail += '；碰撞监控在停车'
            if self._recovery_blocked:
                detail += '；恢复动作被方向性拦截'
            return 'STUCK', detail

        if nav_active:
            if self._nav_status == STATUS_ACCEPTED:
                return 'NAVIGATING', '等待规划器/控制器接管'
            if self._distance_remaining is not None:
                return 'NAVIGATING', (
                    f'距目标 {self._distance_remaining:.2f} m，'
                    f'已恢复 {self._number_of_recoveries} 次')
            return 'NAVIGATING', '沿路径行驶'

        if self._explore_status:
            if self._explore_status in (
                    'exploration_started', 'exploration_in_progress'):
                return 'EXPLORING', '前沿探索进行中'
            if self._explore_status == 'exploration_complete':
                return 'EXPLORE_DONE', '没有更多前沿'
            if self._explore_status == 'exploration_paused':
                return 'IDLE', '探索已暂停'

        return 'IDLE', '等待导航目标'

    def _publish_marker_msg(self, state, recovery_action, stuck_for) -> None:
        marker = Marker()
        marker.header.frame_id = self._marker_frame
        marker.header.stamp = self._now().to_msg()
        marker.ns = 'robot_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = self._marker_height
        marker.scale.z = 0.22
        color = MARKER_COLORS.get(state, MARKER_COLORS['IDLE'])
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0

        text = state
        if recovery_action:
            text += f' | {recovery_action}'
        if state == 'STUCK':
            text += f' {stuck_for:.0f}s'
        marker.text = text

        arr = MarkerArray()
        arr.markers.append(marker)
        self._marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
