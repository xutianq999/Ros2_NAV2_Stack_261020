#!/usr/bin/env python3
"""Route RViz goals to either exploration guidance or manual Nav2 navigation.

Why this node exists
--------------------
RViz's "2D Goal Pose" tool publishes `/goal_pose`, which both bt_navigator and
exploration_ui can consume. In exploration the tool was remapped straight to
`/exploration/guide_pose`, so it could only steer frontier selection: it could
not send a normal navigation goal, and the user could not take over manually.

This node makes the same tool context-sensitive:

    RViz 2D Goal Pose -> /exploration/input_goal
        mode = auto   -> /exploration/guide_pose  (exploration target)
        mode = manual -> /goal_pose               (Nav2 goal)

Switching modes also drives `/explore/resume`, so manual mode cancels and stops
frontier exploration; auto mode resumes it.

Use the same tool for both:
  auto:   ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: true}"
  manual: ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: false}"

Services are available too:
  ros2 service call /exploration_mode/enable  std_srvs/srv/Trigger
  ros2 service call /exploration_mode/disable std_srvs/srv/Trigger
  ros2 service call /exploration_mode/toggle  std_srvs/srv/Trigger
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ExplorationMode(Node):
    """Runtime AUTO/MANUAL switch between exploration and manual navigation."""

    def __init__(self) -> None:
        super().__init__('exploration_mode')

        self.declare_parameter('input_topic', '/exploration/input_goal')
        self.declare_parameter('guide_topic', '/exploration/guide_pose')
        self.declare_parameter('nav_goal_topic', '/goal_pose')
        self.declare_parameter('resume_topic', '/explore/resume')
        self.declare_parameter('mode_command_topic', '/exploration/mode')
        self.declare_parameter('mode_state_topic', '/exploration/enabled')
        self.declare_parameter('startup_mode', 'auto')
        self.declare_parameter('stop_explore_on_manual', True)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._guide_topic = str(self.get_parameter('guide_topic').value)
        self._nav_goal_topic = str(self.get_parameter('nav_goal_topic').value)
        self._resume_topic = str(self.get_parameter('resume_topic').value)
        self._command_topic = str(
            self.get_parameter('mode_command_topic').value)
        self._state_topic = str(self.get_parameter('mode_state_topic').value)
        self._stop_on_manual = bool(
            self.get_parameter('stop_explore_on_manual').value)
        startup = str(self.get_parameter('startup_mode').value).strip().lower()
        self._enabled = startup not in ('manual', 'false', '0', 'off')

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._guide_pub = self.create_publisher(
            PoseStamped, self._guide_topic, 10)
        self._nav_pub = self.create_publisher(
            PoseStamped, self._nav_goal_topic, 10)
        self._resume_pub = self.create_publisher(
            Bool, self._resume_topic, 10)
        self._state_pub = self.create_publisher(
            Bool, self._state_topic, latched)

        self.create_subscription(
            PoseStamped, self._input_topic, self._on_goal, 10)
        self.create_subscription(
            Bool, self._command_topic, self._on_mode_command, 10)
        self.create_service(Trigger, '~/enable', self._on_enable)
        self.create_service(Trigger, '~/disable', self._on_disable)
        self.create_service(Trigger, '~/toggle', self._on_toggle)

        # Publish the initial state and synchronise explore.
        self._publish_state()
        if self._enabled or self._stop_on_manual:
            self._resume_pub.publish(Bool(data=self._enabled))

        self.get_logger().info(
            f'Exploration mode: {"AUTO" if self._enabled else "MANUAL"} '
            f'({self._input_topic} -> '
            f'{"guide" if self._enabled else "Nav2 goal"}); '
            f'commands on {self._command_topic}, state on {self._state_topic}')

    def _publish_state(self) -> None:
        self._state_pub.publish(Bool(data=self._enabled))

    def _set_enabled(self, enabled: bool, source: str) -> None:
        changed = enabled != self._enabled
        self._enabled = enabled
        self._publish_state()
        if changed:
            if enabled or self._stop_on_manual:
                self._resume_pub.publish(Bool(data=enabled))
            self.get_logger().info(
                f'Exploration mode switched to '
                f'{"AUTO" if enabled else "MANUAL"} ({source}).')
        else:
            self.get_logger().debug(
                f'Exploration mode already {"AUTO" if enabled else "MANUAL"}.')

    def _on_mode_command(self, msg: Bool) -> None:
        self._set_enabled(bool(msg.data), 'command')

    def _on_goal(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'
        if self._enabled:
            self._guide_pub.publish(msg)
            self.get_logger().info(
                f'Guide: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}) in {msg.header.frame_id}')
        else:
            self._nav_pub.publish(msg)
            # Do not publish /explore/resume false here: Explore::stop()
            # cancels all NavigateToPose goals, which would cancel the manual
            # goal that was just published. Manual mode already stopped
            # exploration when the mode changed.
            self.get_logger().info(
                f'Manual goal: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}) in {msg.header.frame_id}')

    def _on_enable(self, request, response):
        del request
        self._set_enabled(True, 'service')
        response.success = True
        response.message = 'Exploration mode: AUTO'
        return response

    def _on_disable(self, request, response):
        del request
        self._set_enabled(False, 'service')
        response.success = True
        response.message = 'Exploration mode: MANUAL'
        return response

    def _on_toggle(self, request, response):
        del request
        self._set_enabled(not self._enabled, 'service')
        response.success = True
        response.message = (
            'Exploration mode: AUTO' if self._enabled else
            'Exploration mode: MANUAL')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationMode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
