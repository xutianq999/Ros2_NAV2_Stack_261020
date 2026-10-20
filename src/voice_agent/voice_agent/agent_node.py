"""ROS 2 机器人动作节点：把语音/大模型指令落到 /cmd_vel 和 Nav2。

安全要点：
- 手动运动优先发布到 /cmd_vel_teleop（安全层 twist_mux 的遥控输入），只有在
  没有 twist_mux 订阅者时才回退到 /cmd_vel，避免和 deadman 抢同一个话题；
- 任何运动都有硬超时、速度上限、里程计闭环（可用时）和可选的 /scan 障碍门控；
- “停止”会连续补发零速，避免只发一帧；急停会额外写 /safety/estop 锁存；
- 导航与手动运动互斥，新指令会取消上一动作。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .intent_parser import parse_command
from .waypoints import Waypoint, WaypointStore


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RobotAgent(Node):
    """语音 Agent 的执行端，同时提供 HTTP 层需要的同步接口。"""

    DEFAULT_WAYPOINTS_NAME = "waypoints.yaml"

    def __init__(self) -> None:
        super().__init__("voice_agent_node")
        self._declare_parameters()

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.use_scan_guard = bool(self.get_parameter("use_scan_guard").value)
        self.require_scan_for_motion = bool(
            self.get_parameter("require_scan_for_motion").value
        )
        self.scan_stop_distance = float(self.get_parameter("scan_stop_distance").value)
        self.scan_front_half_angle = math.radians(
            float(self.get_parameter("scan_front_half_angle_deg").value)
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.default_linear_speed = float(
            self.get_parameter("default_linear_speed").value
        )
        self.default_angular_speed = float(
            self.get_parameter("default_angular_speed").value
        )
        self.default_forward_distance = float(
            self.get_parameter("default_forward_distance").value
        )
        self.default_backward_distance = float(
            self.get_parameter("default_backward_distance").value
        )
        self.default_turn_angle = float(
            self.get_parameter("default_turn_angle_deg").value
        )
        self.max_distance = float(self.get_parameter("max_distance_m").value)
        self.max_turn_angle = float(self.get_parameter("max_turn_angle_deg").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.status_rate = float(self.get_parameter("status_rate").value)
        self.motion_rate = float(self.get_parameter("motion_rate").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)
        self.estop_active_topic = str(self.get_parameter("estop_active_topic").value)
        self.autonomy_blocked_topic = str(
            self.get_parameter("autonomy_blocked_topic").value
        )
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        waypoints_file = str(self.get_parameter("waypoints_file").value)
        if not waypoints_file:
            waypoints_file = self._default_waypoints_file()
        self.waypoints = WaypointStore(waypoints_file)

        self._lock = threading.RLock()
        self._mode = "idle"
        self._detail = "待命"
        self._motion: Optional[Dict[str, Any]] = None
        self._zero_until = 0.0
        self._last_cmd_topic = ""
        self._last_odom_pose = None
        self._last_odom_time = 0.0
        self._last_scan = None
        self._last_scan_time = 0.0
        self._estop = False
        self._autonomy_blocked = False
        self._nav_target: Optional[Waypoint] = None
        self._nav_goal_handle = None
        self._nav_feedback: Dict[str, Any] = {}
        self._nav_generation = 0
        self._last_reply = ""

        # 两个手动运动发布器：只在需要时选一个使用，绝不双发。
        self._pub_cmd_direct = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_cmd_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self._pub_cmd_custom = None
        if self.cmd_vel_topic not in ("auto", "/cmd_vel", "/cmd_vel_teleop"):
            self._pub_cmd_custom = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self._pub_estop = self.create_publisher(Bool, self.estop_topic, 10)
        self._pub_reply = self.create_publisher(String, "/voice_agent/reply", 10)
        self._pub_status = self.create_publisher(String, "/voice_agent/status", 10)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(
            String, "/voice_agent/text_command", self._text_command_cb, 10
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, self.estop_active_topic, self._estop_active_cb, latched_qos
        )
        self.create_subscription(
            Bool, self.autonomy_blocked_topic, self._autonomy_blocked_cb, 10
        )

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(1.0 / max(self.motion_rate, 1.0), self._motion_tick)
        self.create_timer(1.0 / max(self.status_rate, 0.5), self._status_tick)

        self.get_logger().info(
            "voice_agent 已启动: cmd_vel=%s, odom=%s, scan=%s, waypoints=%s"
            % (
                self.cmd_vel_topic,
                self.odom_topic,
                self.scan_topic,
                self.waypoints.path,
            )
        )
        if self.waypoints.load_error:
            self.get_logger().warning(self.waypoints.load_error)

    # ------------------------------------------------------------------
    # 参数与工具
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        defaults = {
            "cmd_vel_topic": "auto",
            "odom_topic": "/wheel_odom",
            "scan_topic": "/scan",
            "use_scan_guard": True,
            "require_scan_for_motion": False,
            "scan_stop_distance": 0.35,
            "scan_front_half_angle_deg": 35.0,
            "scan_timeout": 0.5,
            "max_linear_speed": 0.25,
            "max_angular_speed": 0.8,
            "default_linear_speed": 0.15,
            "default_angular_speed": 0.5,
            "default_forward_distance": 0.5,
            "default_backward_distance": 0.3,
            "default_turn_angle_deg": 45.0,
            "max_distance_m": 2.0,
            "max_turn_angle_deg": 180.0,
            "odom_timeout": 0.5,
            "waypoints_file": "",
            "frame_id": "map",
            "status_rate": 2.0,
            "motion_rate": 20.0,
            "estop_topic": "/safety/estop",
            "estop_active_topic": "/safety/estop_active",
            "autonomy_blocked_topic": "/safety/autonomy_blocked",
            "host": os.getenv("HOST", "0.0.0.0"),
            "port": int(os.getenv("PORT", "8000")),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _default_waypoints_file(self) -> str:
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory

            me_share = get_package_share_directory("me_nav2_bringup")
            ws_root = os.path.abspath(
                os.path.join(me_share, "..", "..", "..", "..")
            )
            candidates.append(
                os.path.join(ws_root, "src", "me_nav2_bringup", "config",
                             self.DEFAULT_WAYPOINTS_NAME)
            )
            candidates.append(
                os.path.join(me_share, "config", self.DEFAULT_WAYPOINTS_NAME)
            )
            own_share = get_package_share_directory("voice_agent")
            candidates.append(
                os.path.join(own_share, "config", self.DEFAULT_WAYPOINTS_NAME)
            )
        except Exception:
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(here, "..", "config", self.DEFAULT_WAYPOINTS_NAME))
        for path in candidates:
            if path and os.path.isfile(path):
                return os.path.abspath(path)
        return candidates[-1]

    def _cmd_publisher(self):
        if self._pub_cmd_custom is not None:
            return self._pub_cmd_custom, self.cmd_vel_topic
        if self.cmd_vel_topic == "/cmd_vel_teleop":
            return self._pub_cmd_teleop, "/cmd_vel_teleop"
        if self.cmd_vel_topic == "/cmd_vel":
            # 安全层启用后 /cmd_vel 由 deadman 独占。直发会绕过 twist_mux、
            # 急停锁、看门狗锁和恢复守卫，只应在没有安全层的调试场景使用。
            self.get_logger().warn(
                'cmd_vel_topic=/cmd_vel 会绕过安全层；'
                '启用 nav2_safety 时请改用 /cmd_vel_teleop',
                throttle_duration_sec=10.0)
            return self._pub_cmd_direct, "/cmd_vel"
        # auto：固定走遥控输入。安全层启用时 /cmd_vel 由 deadman 独占，
        # 不能再直发（会绕过整条安全链）；没有安全层时 /cmd_vel_teleop
        # 没有订阅者，但那时本来就不该用 auto。
        return self._pub_cmd_teleop, "/cmd_vel_teleop"

    def _publish_twist_locked(self, twist: Twist) -> None:
        pub, topic = self._cmd_publisher()
        if topic != self._last_cmd_topic:
            self.get_logger().info(f"运动输出话题切换为 {topic}")
            self._last_cmd_topic = topic
        pub.publish(twist)

    def _publish_zero_locked(self) -> None:
        self._publish_twist_locked(Twist())

    def _reply(self, text: str) -> str:
        self._last_reply = text
        self._pub_reply.publish(String(data=text))
        return text

    def _result(self, ok: bool, reply: str, action: str, **extra) -> Dict[str, Any]:
        self._reply(reply)
        result = {"handled": True, "ok": ok, "action": action, "reply": reply}
        result.update(extra)
        return result

    def _odom_fresh(self) -> bool:
        return (
            self._last_odom_time > 0.0
            and (time.monotonic() - self._last_odom_time) < self.odom_timeout
        )

    def _scan_fresh(self) -> bool:
        return (
            self._last_scan_time > 0.0
            and (time.monotonic() - self._last_scan_time) < self.scan_timeout
        )

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        with self._lock:
            self._last_odom_pose = pose
            self._last_odom_time = time.monotonic()
            if (
                self._mode == "moving"
                and self._motion is not None
                and self._motion.get("start_pose") is None
            ):
                # 指令下发后才收到第一帧里程计，用它作为起点
                self._motion["start_pose"] = pose

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self._last_scan = msg
            self._last_scan_time = time.monotonic()

    def _estop_active_cb(self, msg: Bool) -> None:
        with self._lock:
            self._estop = bool(msg.data)
            if self._estop:
                self._stop_motion_locked()
                self._cancel_nav_locked()
                self._zero_until = time.monotonic() + 0.5
        if self._estop:
            self.get_logger().warning("检测到安全层急停锁存，运动指令已屏蔽")

    def _autonomy_blocked_cb(self, msg: Bool) -> None:
        self._autonomy_blocked = bool(msg.data)

    def _text_command_cb(self, msg: String) -> None:
        self.handle_text(msg.data, source="ros_topic")

    # ------------------------------------------------------------------
    # 手动运动
    # ------------------------------------------------------------------
    def _scan_min_range(self, direction: str) -> Optional[float]:
        msg = self._last_scan
        if msg is None:
            return None
        target = 0.0 if direction == "forward" else math.pi
        best = math.inf
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if msg.range_min > 0.0 and distance < msg.range_min:
                continue
            if msg.range_max > 0.0 and distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(wrap_angle(angle - target)) <= self.scan_front_half_angle:
                best = min(best, distance)
        return None if math.isinf(best) else best

    def start_move(
        self,
        direction: str,
        distance_m: Optional[float] = None,
        angle_deg: Optional[float] = None,
        speed: Optional[float] = None,
        speed_scale: float = 1.0,
        source: str = "voice",
    ) -> Dict[str, Any]:
        direction = self._normalize_direction(direction)
        if direction is None:
            return self._result(False, "不支持这个运动方向。", "move")

        with self._lock:
            if self._estop:
                return self._result(
                    False, "急停锁存中，请先说“解除急停”。", "move"
                )
            if (
                self.use_scan_guard
                and self.require_scan_for_motion
                and not self._scan_fresh()
            ):
                return self._result(
                    False, "没有收到 /scan 障碍数据，为安全起见暂不移动。", "move"
                )

            self._stop_motion_locked()
            self._cancel_nav_locked()

            scale = min(max(float(speed_scale or 1.0), 0.3), 1.5)
            now = time.monotonic()
            motion: Dict[str, Any] = {
                "direction": direction,
                "start_time": now,
                "start_pose": self._last_odom_pose if self._odom_fresh() else None,
            }

            if direction in ("forward", "backward"):
                default = (
                    self.default_forward_distance
                    if direction == "forward"
                    else self.default_backward_distance
                )
                raw_distance = default if distance_m is None else float(distance_m)
                distance = min(max(raw_distance, 0.05), self.max_distance)
                linear = float(speed or self.default_linear_speed) * scale
                linear = min(max(linear, 0.03), self.max_linear_speed)
                duration = distance / linear
                motion.update(
                    distance=distance,
                    angle_rad=None,
                    linear=linear if direction == "forward" else -linear,
                    angular=0.0,
                    duration=duration,
                )
                reply = (
                    f"好的，前进 {distance:.2f} 米。"
                    if direction == "forward"
                    else f"好的，后退 {distance:.2f} 米。"
                )
            else:
                raw_angle = self.default_turn_angle if angle_deg is None else float(angle_deg)
                angle = min(max(raw_angle, 5.0), self.max_turn_angle)
                angular = float(speed or self.default_angular_speed) * scale
                angular = min(max(angular, 0.1), self.max_angular_speed)
                duration = math.radians(angle) / angular
                motion.update(
                    distance=None,
                    angle_rad=math.radians(angle),
                    linear=0.0,
                    angular=angular if direction == "left" else -angular,
                    duration=duration,
                )
                reply = (
                    f"好的，左转 {angle:.0f} 度。"
                    if direction == "left"
                    else f"好的，右转 {angle:.0f} 度。"
                )

            # 硬超时：即使里程计没进展也一定会停
            motion["deadline"] = now + motion["duration"] * 2.5 + 1.0
            self._motion = motion
            self._mode = "moving"
            self._detail = reply.strip("。")
            self.get_logger().info(f"手动运动开始({source}): {reply}")
            return self._result(True, reply, "move", motion={
                key: motion[key] for key in ("direction", "distance", "angle_rad")
            })

    @staticmethod
    def _normalize_direction(direction: Optional[str]) -> Optional[str]:
        if not direction:
            return None
        mapping = {
            "forward": "forward", "前进": "forward", "向前": "forward",
            "backward": "backward", "back": "backward", "后退": "backward",
            "向后": "backward", "倒车": "backward",
            "left": "left", "左转": "left", "向左": "left",
            "right": "right", "右转": "right", "向右": "right",
        }
        return mapping.get(str(direction).strip().lower())

    def _stop_motion_locked(self) -> None:
        self._motion = None
        if self._mode == "moving":
            self._mode = "idle"
            self._detail = "已停止"

    def _motion_tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            if self._mode == "moving" and self._motion is not None:
                motion = self._motion
                elapsed = now - motion["start_time"]

                if now >= motion["deadline"]:
                    self._finish_motion_locked("运动超时，已自动停止。")
                    return

                if motion["linear"] != 0.0 and self.use_scan_guard:
                    if self.require_scan_for_motion and not self._scan_fresh():
                        self._finish_motion_locked(
                            "障碍数据中断，已自动停止。"
                        )
                        return
                    if self._scan_fresh():
                        side = "forward" if motion["linear"] > 0.0 else "backward"
                        nearest = self._scan_min_range(side)
                        if nearest is not None and nearest < self.scan_stop_distance:
                            where = "前方" if side == "forward" else "后方"
                            self._finish_motion_locked(
                                f"检测到{where}障碍物，已停止。"
                            )
                            return

                done = False
                progress_ratio = 0.0
                if self._odom_fresh() and motion.get("start_pose") is not None:
                    start = motion["start_pose"]
                    current = self._last_odom_pose
                    if motion["distance"] is not None:
                        traveled = math.hypot(
                            current[0] - start[0], current[1] - start[1]
                        )
                        progress_ratio = traveled / max(motion["distance"], 1e-6)
                        done = progress_ratio >= 0.98
                    else:
                        turned = abs(wrap_angle(current[2] - start[2]))
                        progress_ratio = turned / max(motion["angle_rad"], 1e-6)
                        done = progress_ratio >= 0.98
                else:
                    # 没有里程计就用时间估算
                    progress_ratio = elapsed / max(motion["duration"], 1e-6)
                    done = progress_ratio >= 1.0

                if done:
                    self._finish_motion_locked("动作完成。")
                    return

                # 有里程计但明显没动：可能是被挡住/轮子打滑，提前放弃
                if (
                    self._odom_fresh()
                    and elapsed > motion["duration"] * 1.5
                    and progress_ratio < 0.2
                ):
                    self._finish_motion_locked("移动受阻，已停止。")
                    return

                twist = Twist()
                twist.linear.x = float(motion["linear"])
                twist.angular.z = float(motion["angular"])
                self._publish_twist_locked(twist)
                return

            if now < self._zero_until:
                self._publish_zero_locked()

    def _finish_motion_locked(self, reply: str) -> None:
        self._motion = None
        self._mode = "idle"
        self._detail = reply.rstrip("。")
        self._zero_until = time.monotonic() + 0.4
        self._publish_zero_locked()
        self.get_logger().info(reply)
        self._reply(reply)

    def stop_all(self, reason: str = "stop") -> Dict[str, Any]:
        with self._lock:
            was_active = self._mode != "idle"
            self._mode = "idle"
            self._motion = None
            self._detail = "已停止"
            self._cancel_nav_locked()
            self._zero_until = time.monotonic() + 0.6
            self._publish_zero_locked()
        reply = "已停止。" if was_active else "小车当前已经是停止状态。"
        self.get_logger().info(f"停止指令({reason})")
        return self._result(True, reply, "stop")

    # ------------------------------------------------------------------
    # 急停
    # ------------------------------------------------------------------
    def emergency_stop(self) -> Dict[str, Any]:
        with self._lock:
            self._estop = True
            self._mode = "idle"
            self._motion = None
            self._detail = "急停锁存"
            self._cancel_nav_locked()
            self._zero_until = time.monotonic() + 1.0
            self._publish_zero_locked()
        self._pub_estop.publish(Bool(data=True))
        self.get_logger().warning("软件急停已触发")
        return self._result(
            True, "急停已触发，小车已停止。需要恢复请说“解除急停”。", "estop"
        )

    def release_estop(self) -> Dict[str, Any]:
        self._pub_estop.publish(Bool(data=False))
        with self._lock:
            self._estop = False
            self._detail = "急停已解除"
            self._zero_until = time.monotonic() + 0.3
            self._publish_zero_locked()
        self.get_logger().info("软件急停已解除")
        return self._result(True, "急停已解除，可以继续下指令。", "release_estop")

    # ------------------------------------------------------------------
    # 导航
    # ------------------------------------------------------------------
    def navigate_to(self, point_query: str, source: str = "voice") -> Dict[str, Any]:
        with self._lock:
            if self._estop:
                return self._result(
                    False, "急停锁存中，请先说“解除急停”。", "navigate"
                )
            waypoint = self.waypoints.resolve(point_query)
            if waypoint is None:
                return self._result(
                    False,
                    f"没有找到点位“{point_query}”。可用点位：{self.waypoints.list_text()}",
                    "navigate",
                )
            if not self.nav_client.server_is_ready():
                return self._result(
                    False,
                    "Nav2 导航服务还没有就绪，请先启动导航栈。",
                    "navigate",
                    point=waypoint.as_dict(),
                )

            self._stop_motion_locked()
            self._cancel_nav_locked()
            self._nav_generation += 1
            generation = self._nav_generation
            self._nav_target = waypoint
            self._nav_feedback = {}
            self._mode = "navigating"
            self._detail = f"前往 {waypoint.id} 号点位"
            self._zero_until = time.monotonic() + 0.3
            self._publish_zero_locked()

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.frame_id
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = waypoint.x
            goal.pose.pose.position.y = waypoint.y
            goal.pose.pose.position.z = 0.0
            q = quaternion_from_yaw(waypoint.yaw)
            goal.pose.pose.orientation.x = q[0]
            goal.pose.pose.orientation.y = q[1]
            goal.pose.pose.orientation.z = q[2]
            goal.pose.pose.orientation.w = q[3]

            future = self.nav_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._nav_feedback_cb(
                    generation, feedback
                ),
            )
            future.add_done_callback(
                lambda done: self._nav_goal_response_cb(generation, done)
            )

            reply = f"好的，前往 {waypoint.id} 号点位。"
            if self._autonomy_blocked:
                reply += "注意：安全层当前锁住了自动驾驶，可能不会立即移动。"
            self.get_logger().info(f"导航目标({source}): {reply}")
            return self._result(True, reply, "navigate", point=waypoint.as_dict())

    def _nav_goal_response_cb(self, generation: int, future) -> None:
        if generation != self._nav_generation:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"发送导航目标失败: {exc}")
            with self._lock:
                self._mode = "idle"
                self._nav_target = None
                self._detail = "导航目标发送失败"
            self._reply("导航目标发送失败，请检查 Nav2 状态。")
            return

        if not goal_handle.accepted:
            with self._lock:
                self._mode = "idle"
                self._nav_target = None
                self._detail = "导航目标被拒绝"
            self._reply("导航目标被拒绝，可能是地图或定位还没准备好。")
            return

        with self._lock:
            if generation != self._nav_generation:
                return
            self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._nav_result_cb(generation, done)
        )

    def _nav_feedback_cb(self, generation: int, feedback_msg) -> None:
        if generation != self._nav_generation:
            return
        feedback = feedback_msg.feedback
        with self._lock:
            self._nav_feedback = {
                "distance_remaining": float(feedback.distance_remaining),
                "number_of_recoveries": int(feedback.number_of_recoveries),
                "navigation_time": int(feedback.navigation_time.sec),
                "estimated_time_remaining": int(
                    feedback.estimated_time_remaining.sec
                ),
            }
            target = self._nav_target
            if target is not None:
                self._detail = (
                    f"前往 {target.id} 号点位，剩余 "
                    f"{feedback.distance_remaining:.1f} 米"
                )

    def _nav_result_cb(self, generation: int, future) -> None:
        if generation != self._nav_generation:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f"读取导航结果失败: {exc}")
            status = GoalStatus.STATUS_ABORTED

        with self._lock:
            target = self._nav_target
            point_label = f"{target.id} 号点位" if target is not None else "目标点"
            self._mode = "idle"
            self._nav_goal_handle = None
            self._nav_target = None
            self._nav_feedback = {}

            if status == GoalStatus.STATUS_SUCCEEDED:
                reply = f"已到达{point_label}。"
                self._detail = reply.rstrip("。")
            elif status == GoalStatus.STATUS_CANCELED:
                reply = "导航已取消。"
                self._detail = "导航已取消"
            else:
                reply = f"导航失败或受阻，已停止。目标：{point_label}。"
                self._detail = "导航失败"
        self.get_logger().info(reply)
        self._reply(reply)

    def cancel_navigation(self) -> Dict[str, Any]:
        with self._lock:
            had_nav = self._nav_goal_handle is not None or self._mode == "navigating"
            self._cancel_nav_locked()
            self._detail = "导航已取消"
            self._zero_until = time.monotonic() + 0.4
            self._publish_zero_locked()
        reply = "导航已取消。" if had_nav else "当前没有正在执行的导航。"
        return self._result(True, reply, "cancel_nav")

    def _cancel_nav_locked(self) -> None:
        handle = self._nav_goal_handle
        self._nav_goal_handle = None
        self._nav_target = None
        self._nav_feedback = {}
        self._nav_generation += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f"取消导航目标失败: {exc}")
        if self._mode == "navigating":
            self._mode = "idle"

    # ------------------------------------------------------------------
    # 文本指令入口
    # ------------------------------------------------------------------
    def handle_text(self, text: str, source: str = "voice") -> Dict[str, Any]:
        command = parse_command(text)
        if command is None:
            return {"handled": False, "action": "chat", "ok": True, "reply": ""}

        if command.action == "move":
            return self.start_move(
                direction=command.direction or "",
                distance_m=command.distance_m,
                angle_deg=command.angle_deg,
                speed_scale=command.speed_scale,
                source=source,
            )
        if command.action == "stop":
            return self.stop_all(reason=f"{source}:stop")
        if command.action == "estop":
            return self.emergency_stop()
        if command.action == "release_estop":
            return self.release_estop()
        if command.action == "cancel_nav":
            return self.cancel_navigation()
        if command.action == "navigate":
            return self.navigate_to(command.point or "", source=source)
        if command.action == "list_waypoints":
            reply = f"可用的点位有：{self.waypoints.list_text()}"
            return self._result(True, reply, "list_waypoints")
        if command.action == "status":
            return self._result(True, self.describe_status(), "status")
        if command.action == "help":
            reply = (
                "你可以说：前进、后退、左转、右转、停止、急停，"
                "或者说去一号点位、取消导航、有哪些点位。"
            )
            return self._result(True, reply, "help")
        if command.action == "unsupported":
            return self._result(
                False,
                "小车是差速底盘，不能横向平移。我可以左转或右转。",
                "unsupported",
            )
        return {"handled": False, "action": "chat", "ok": True, "reply": ""}

    def describe_status(self) -> str:
        with self._lock:
            if self._mode == "navigating" and self._nav_target is not None:
                remaining = self._nav_feedback.get("distance_remaining")
                detail = f"正在前往 {self._nav_target.id} 号点位"
                if remaining is not None:
                    detail += f"，剩余约 {remaining:.1f} 米"
            elif self._mode == "moving":
                detail = self._detail
            elif self._estop:
                detail = "急停锁存中"
            else:
                detail = "待命中"

            if self._last_odom_pose is not None:
                x, y, _ = self._last_odom_pose
                detail += f"。当前位置 x={x:.2f}，y={y:.2f}"
            if not self._odom_fresh():
                detail += "。里程计暂时没有数据"
            if self._autonomy_blocked:
                detail += "。自动驾驶当前被安全层锁住"
            return detail + "。"

    # ------------------------------------------------------------------
    # 状态发布
    # ------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            target = self._nav_target
            status = {
                "mode": self._mode,
                "detail": self._detail,
                "estop": self._estop,
                "autonomy_blocked": self._autonomy_blocked,
                "odom_ok": self._odom_fresh(),
                "scan_ok": self._scan_fresh(),
                "cmd_vel_topic": self._last_cmd_topic or self.cmd_vel_topic,
                "odom_topic": self.odom_topic,
                "scan_topic": self.scan_topic,
                "waypoints_file": self.waypoints.path,
                "waypoint_error": self.waypoints.load_error,
                "point": target.as_dict() if target is not None else None,
                "nav_feedback": dict(self._nav_feedback),
                "last_reply": self._last_reply,
            }
            if self._last_odom_pose is not None:
                status["pose"] = {
                    "x": self._last_odom_pose[0],
                    "y": self._last_odom_pose[1],
                    "yaw": self._last_odom_pose[2],
                }
            return status

    def _status_tick(self) -> None:
        payload = json.dumps(self.get_status(), ensure_ascii=False)
        self._pub_status.publish(String(data=payload))

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        try:
            with self._lock:
                self._mode = "idle"
                self._motion = None
                self._cancel_nav_locked()
                self._zero_until = time.monotonic() + 0.5
            for _ in range(12):
                self._publish_zero_locked()
                time.sleep(0.02)
        except Exception as exc:
            self.get_logger().warning(f"关闭时发送零速失败: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
