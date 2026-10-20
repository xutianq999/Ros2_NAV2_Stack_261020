"""Recovery-only low-speed swept-footprint guard and command ownership.

Close obstacles may stay equally close or recede. Approaching motions keep a
small clearance and slow down before stopping. Never changes arc curvature.
The mux still applies software emergency-stop and localization locks.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import BackUp, DriveOnHeading, Spin, Wait
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from nav2_safety.recovery_activity import RecoveryActivity
from nav2_safety.recovery_motion import limit_motion


class RecoveryGuard(Node):
    def __init__(self):
        super().__init__('recovery_guard')
        defaults = {
            'input_topic': 'cmd_vel_recovery',
            'output_topic': 'cmd_vel_recovery_checked', 'scan_topic': '/scan',
            'base_frame': 'base_footprint', 'half_length': 0.345, 'half_width': 0.285,
            'min_clearance': 0.025, 'min_points': 2, 'lookahead_time': 0.5,
            'min_linear': 0.02, 'min_angular': 0.04,
            'scan_timeout': 0.5, 'input_timeout': 0.3, 'publish_rate': 20.0,
            'max_linear': 0.15, 'max_angular': 0.8,
            'blocked_topic': '/safety/recovery_blocked',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        gp = lambda key: self.get_parameter(key).value
        positive = ('half_length', 'half_width', 'min_clearance', 'lookahead_time',
                    'min_linear', 'min_angular', 'scan_timeout', 'input_timeout',
                    'publish_rate', 'max_linear', 'max_angular')
        if any(not math.isfinite(float(gp(k))) or gp(k) <= 0 for k in positive):
            raise ValueError('Recovery motion parameters must be finite and positive')
        if (gp('min_points') < 1 or gp('max_linear') > 0.15 or gp('max_angular') > 0.8
                or gp('min_clearance') < 0.01 or gp('lookahead_time') < 0.3
                or gp('min_linear') > gp('max_linear')
                or gp('min_angular') > gp('max_angular')):
            raise ValueError('Recovery motion parameters exceed low-speed limits')
        self._base = str(gp('base_frame'))
        self._scan_timeout = float(gp('scan_timeout'))
        self._input_timeout = float(gp('input_timeout'))
        self._policy = {key: gp(key) for key in (
            'half_length', 'half_width', 'min_points', 'max_linear', 'max_angular',
            'min_linear', 'min_angular')}
        self._policy.update(clearance=gp('min_clearance'), horizon=gp('lookahead_time'))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pts = None
        self._scan_wall = self._cmd_wall = 0.0
        self._cmd = Twist()
        self._last_state = None
        self._last_status = self._last_owner_probe = 0.0
        self._status_seen = {}
        self._result_requests = {}
        self._activity = RecoveryActivity()
        self._pub = self.create_publisher(Twist, str(gp('output_topic')), 10)
        self._blocked_pub = self.create_publisher(Bool, str(gp('blocked_topic')), 10)
        self._active_pub = self.create_publisher(Bool, '/safety/recovery_active', 1)
        self._status_pub = self.create_publisher(String, '/safety/recovery_status', 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        types = {'smart_recovery': BackUp, 'backup': BackUp, 'spin': Spin,
                 'drive_on_heading': DriveOnHeading, 'wait': Wait}
        self._result_clients = {name: self.create_client(
            typ.Impl.GetResultService, f'/{name}/_action/get_result')
            for name, typ in types.items()}
        self._result_types = {name: typ.Impl.GetResultService for name, typ in types.items()}
        self._status_subs = [self.create_subscription(
            GoalStatusArray, f'/{name}/_action/status',
            lambda msg, name=name: self._on_action_status(name, msg), status_qos)
            for name in types]
        self._status_topics = {name: sub.topic_name
                               for name, sub in zip(types, self._status_subs)}
        self.create_subscription(Twist, str(gp('input_topic')), self._on_cmd, 10)
        self.create_subscription(LaserScan, str(gp('scan_topic')), self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('publish_rate')), self._tick)
        self.get_logger().info(
            '脱困守卫：整车扫掠 + 2.5cm 默认间隙 + 等比例降速；'
            '贴墙可沿墙/远离障碍，动作结果确认后释放通道')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self._cmd, self._cmd_wall = msg, self._now()

    def _on_action_status(self, name, msg):
        self._status_seen[name] = time.monotonic()
        self._activity.update(name, msg.status_list, self._status_seen[name])
        for key in self._activity.goals[name]:
            self._request_result(name, key)

    def _request_result(self, name, key):
        pair = (name, key)
        client = self._result_clients[name]
        if pair in self._result_requests or not client.service_is_ready():
            return
        request = self._result_types[name].Request()
        request.goal_id.uuid = list(key)
        future = client.call_async(request)
        self._result_requests[pair] = future
        future.add_done_callback(lambda f: self._on_result(name, key, f))

    def _on_result(self, name, key, future):
        self._result_requests.pop((name, key), None)
        if future.cancelled():
            return
        try:
            response = future.result()
            # UNKNOWN(0) is not terminal: the server may not have registered
            # the goal yet. Keep the lease and retry; it expires after
            # RecoveryActivity.timeout if the server is really gone.
            if response.status in (4, 5, 6):
                self._activity.finish(name, key)
        except Exception as exc:
            self.get_logger().warn(f'脱困结果确认失败: {exc}', throttle_duration_sec=2.0)

    def _probe_owners(self):
        now = time.monotonic()
        if now - self._last_owner_probe < 0.2:
            return
        self._last_owner_probe = now
        for name, goals in list(self._activity.goals.items()):
            if (goals and now - self._status_seen.get(name, now) > 0.75
                    and self.count_publishers(self._status_topics[name]) == 0):
                self._activity.server_gone(name)
            # DDS service discovery may finish after the one active-status event.
            for key in list(goals):
                self._request_result(name, key)
        for pair, future in list(self._result_requests.items()):
            if pair[1] not in self._activity.goals.get(pair[0], {}):
                self._result_requests.pop(pair, None)
                self._result_clients[pair[0]].remove_pending_request(future)
                future.cancel()

    def _on_scan(self, msg: LaserScan) -> None:
        metadata = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if (not all(math.isfinite(v) for v in metadata)
                or msg.angle_increment <= 0 or msg.range_max <= msg.range_min):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self._now() - stamp
        if not math.isfinite(age) or age < -0.1 or age > self._scan_timeout:
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        if not ok.any():
            # 整帧没有有效回波：不刷新时间戳，也不清空上一帧的点。
            # 连续 scan_timeout 没有有效观测后，_tick 会 fail-closed 输出零速；
            # 若在这里刷新时间戳，空扫描会被一直当成"新鲜且畅通"。
            return
        pts = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.0))
        except Exception as exc:  # TF 不通 -> 这一帧作废，等超时后 fail-closed
            self.get_logger().warn(
                f'TF {msg.header.frame_id}->{self._base} 不可用: {exc}',
                throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        # 只用 yaw：雷达相对底盘的俯仰/横滚对 2D 切片的 x,y 影响可忽略
        x = c * pts[:, 0] - s * pts[:, 1] + t.x
        y = s * pts[:, 0] + c * pts[:, 1] + t.y
        self._pts = np.stack([x, y], axis=1)
        self._scan_wall = self._now()

    def _status(self, state, detail, scale=0.0):
        now = time.monotonic()
        signature = (state, detail, round(scale, 3))
        if signature == self._last_state and now - self._last_status < 1.0:
            return
        if signature != self._last_state:
            self.get_logger().info(f'脱困通道 [{state}] {detail}')
        self._last_state, self._last_status = signature, now
        self._status_pub.publish(String(data=json.dumps(
            dict(state=state, detail=detail, scale=round(scale, 3)), ensure_ascii=False)))

    def _tick(self):
        self._probe_owners()
        now = self._now()
        active_count = self._activity.count(time.monotonic())
        self._active_pub.publish(Bool(data=active_count > 0))
        out = Twist()
        if not 0 <= now - self._cmd_wall <= self._input_timeout:
            if active_count:
                self._pub.publish(out)
                self._status('waiting', '脱困动作有效，等待速度指令；导航保持让行')
            else:
                self._status('idle', '脱困通道已释放')
            self._blocked_pub.publish(Bool(data=False))
            return
        values = (self._cmd.linear.x, self._cmd.linear.y, self._cmd.angular.z)
        if not all(math.isfinite(v) for v in values) or abs(values[1]) > 1e-3:
            state, detail, blocked, scale = 'invalid', '不支持的速度指令，停车', True, 0.0
        elif active_count > 1:
            state, detail, blocked, scale = 'conflict', '多个恢复动作同时有效，等待动作交接', True, 0.0
        elif self._pts is None or not 0 <= now - self._scan_wall <= self._scan_timeout:
            state, detail, scale = 'sensor_stale', '扫描过期/空帧/TF 不可用，停车', 0.0
            blocked = abs(values[0]) > 1e-3 or abs(values[2]) > 1e-3
        else:
            out.linear.x, out.angular.z, scale = limit_motion(
                self._pts, values[0], values[2], **self._policy)
            requested = abs(values[0]) > 1e-3 or abs(values[2]) > 1e-3
            blocked = requested and scale == 0
            if blocked:
                state, detail = 'obstacle', '整车扫掠会碰撞或缩小已有近距离间隙，停车并换路线'
            elif requested and scale < 0.999:
                state, detail = 'creeping', f'空间受限，保持路线并降速至 {scale:.0%}'
            elif requested:
                state, detail = 'moving', '扫掠安全，允许低速脱困'
            else:
                state, detail = 'stopped', '恢复动作主动停车/交接'
        self._pub.publish(out)
        self._blocked_pub.publish(Bool(data=blocked))
        self._status(state, detail, scale)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
