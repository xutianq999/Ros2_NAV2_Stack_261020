#!/usr/bin/env python3
"""Visual exploration helper: boundary polygon + direction guide.

Workflow (RViz):
  1. Use the "Publish Point" tool to click the boundary vertices in order.
  2. Commit the polygon:
       ros2 service call /exploration_ui/commit std_srvs/srv/Trigger
     explore_lite then only considers frontiers whose centroid is inside it.
  3. Use the "2D Goal Pose" tool to point at a target. The RViz goal goes to
     exploration_mode, which routes it by mode:
       AUTO   -> /exploration/guide_pose -> this node -> explore_lite
       MANUAL -> /goal_pose -> Nav2 (exploration is stopped)
     In AUTO mode explore_lite also uses the guide as a direct high-priority
     target (guide_navigation=true), not only as a direction filter.
  4. Switch modes:
       ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: false}"
       ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: true}"
  5. Clear the boundary / guide:
       ros2 service call /exploration_ui/clear std_srvs/srv/Trigger
       ros2 service call /exploration_ui/clear_guide std_srvs/srv/Trigger

The boundary can be saved to / loaded from YAML:
       ros2 service call /exploration_ui/save std_srvs/srv/Trigger
       ros2 service call /exploration_ui/load std_srvs/srv/Trigger
"""

from __future__ import annotations

import os

import rclpy
import yaml
from geometry_msgs.msg import Point, Point32, PointStamped, PolygonStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ExplorationUI(Node):
    def __init__(self) -> None:
        super().__init__('exploration_ui')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('click_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('bounds_topic', '/exploration/bounds')
        self.declare_parameter('guide_topic', '/exploration/guide')
        self.declare_parameter('guide_clear_topic', '/exploration/guide_clear')
        self.declare_parameter('bounds_marker_topic', '/exploration/bounds_markers')
        self.declare_parameter('guide_marker_topic', '/exploration/guide_markers')
        self.declare_parameter('min_points', 3)
        self.declare_parameter('bounds_file', '')
        self.declare_parameter('auto_load', False)

        self._frame = str(self.get_parameter('frame_id').value)
        self._click_topic = str(self.get_parameter('click_topic').value)
        self._goal_topic = str(self.get_parameter('goal_topic').value)
        self._bounds_topic = str(self.get_parameter('bounds_topic').value)
        self._guide_topic = str(self.get_parameter('guide_topic').value)
        self._guide_clear_topic = str(self.get_parameter('guide_clear_topic').value)
        self._bounds_marker_topic = str(
            self.get_parameter('bounds_marker_topic').value)
        self._guide_marker_topic = str(
            self.get_parameter('guide_marker_topic').value)
        self._min_points = max(3, int(self.get_parameter('min_points').value))
        self._bounds_file = str(self.get_parameter('bounds_file').value)
        self._auto_load = bool(self.get_parameter('auto_load').value)

        self._bounds: list[tuple[float, float]] = []
        self._guide: PoseStamped | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._bounds_pub = self.create_publisher(
            PolygonStamped, self._bounds_topic, latched)
        self._guide_pub = self.create_publisher(
            PoseStamped, self._guide_topic, latched)
        self._guide_clear_pub = self.create_publisher(
            Empty, self._guide_clear_topic, 10)
        self._bounds_marker_pub = self.create_publisher(
            MarkerArray, self._bounds_marker_topic, latched)
        self._guide_marker_pub = self.create_publisher(
            MarkerArray, self._guide_marker_topic, latched)

        self.create_subscription(
            PointStamped, self._click_topic, self._on_click, 10)
        self.create_subscription(
            PoseStamped, self._goal_topic, self._on_goal, 10)

        self.create_service(Trigger, '~/commit', self._on_commit)
        self.create_service(Trigger, '~/clear', self._on_clear)
        self.create_service(Trigger, '~/save', self._on_save)
        self.create_service(Trigger, '~/load', self._on_load)
        self.create_service(Trigger, '~/clear_guide', self._on_clear_guide)

        if self._auto_load:
            self._load_file(quiet=True)

        self._publish_bounds_markers()
        self._publish_guide_markers()
        self.get_logger().info(
            f'Exploration UI ready (frame={self._frame}). '
            f'Click vertices on {self._click_topic}, then call '
            '/exploration_ui/commit. Click a direction on '
            f'{self._goal_topic} for guidance.')

    # ------------------------------------------------------------------ input

    def _on_click(self, msg: PointStamped) -> None:
        point = self._transform_point(msg)
        if point is None:
            return
        self._bounds.append(point)
        self._publish_bounds_markers()
        self.get_logger().info(
            f'Boundary vertex {len(self._bounds)}: '
            f'({point[0]:.2f}, {point[1]:.2f})')

    def _on_goal(self, msg: PoseStamped) -> None:
        pose = self._transform_pose(msg)
        if pose is None:
            return
        self._guide = pose
        self._guide_pub.publish(pose)
        self._publish_guide_markers()
        self.get_logger().info(
            f'Guide set to ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}) in frame {self._frame}')

    # ------------------------------------------------------------- transforms

    def _transform_point(self, msg: PointStamped) -> tuple[float, float] | None:
        frame = msg.header.frame_id or self._frame
        if frame == self._frame:
            return float(msg.point.x), float(msg.point.y)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame, frame, Time())
            out = do_transform_point(msg, transform)
            return float(out.point.x), float(out.point.y)
        except Exception as exc:  # noqa: BLE001 - TF errors are expected at startup
            self.get_logger().warning(
                f'Cannot transform clicked point from {frame} to '
                f'{self._frame}: {exc}', throttle_duration_sec=5.0)
            return None

    def _transform_pose(self, msg: PoseStamped) -> PoseStamped | None:
        frame = msg.header.frame_id or self._frame
        try:
            if frame == self._frame:
                out = PoseStamped()
                out.header.frame_id = self._frame
                out.pose = msg.pose
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._frame, frame, Time())
                out = do_transform_pose(msg, transform)
                out.header.frame_id = self._frame
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as exc:  # noqa: BLE001 - TF errors are expected at startup
            self.get_logger().warning(
                f'Cannot transform guide pose from {frame} to '
                f'{self._frame}: {exc}', throttle_duration_sec=5.0)
            return None

    # --------------------------------------------------------------- services

    def _on_commit(self, request, response):
        del request
        if len(self._bounds) < self._min_points:
            response.success = False
            response.message = (
                f'Need at least {self._min_points} vertices, '
                f'got {len(self._bounds)}')
            return response
        self._publish_bounds()
        self._publish_bounds_markers()
        self._save_file(quiet=True)
        response.success = True
        response.message = f'Committed {len(self._bounds)} boundary vertices'
        return response

    def _on_clear(self, request, response):
        del request
        self._bounds.clear()
        self._publish_bounds()
        self._publish_bounds_markers()
        response.success = True
        response.message = 'Exploration boundary cleared (unlimited)'
        return response

    def _on_save(self, request, response):
        del request
        if self._save_file(quiet=False):
            response.success = True
            response.message = f'Saved boundary to {self._bounds_file}'
        else:
            response.success = False
            response.message = (
                'Cannot save boundary: bounds_file is empty or write failed')
        return response

    def _on_load(self, request, response):
        del request
        if self._load_file(quiet=False):
            response.success = True
            response.message = f'Loaded {len(self._bounds)} vertices'
        else:
            response.success = False
            response.message = 'Cannot load boundary from bounds_file'
        return response

    def _on_clear_guide(self, request, response):
        del request
        self._guide = None
        self._guide_clear_pub.publish(Empty())
        self._publish_guide_markers()
        response.success = True
        response.message = 'Direction guide cleared'
        return response

    # --------------------------------------------------------------- storage

    def _save_file(self, quiet: bool) -> bool:
        if not self._bounds_file:
            return False
        if len(self._bounds) < self._min_points:
            if not quiet:
                self.get_logger().info(
                    'No boundary to save (fewer than '
                    f'{self._min_points} vertices).')
            return False
        try:
            os.makedirs(
                os.path.dirname(os.path.abspath(self._bounds_file)),
                exist_ok=True)
            with open(self._bounds_file, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    {
                        'frame': self._frame,
                        'points': [[x, y] for x, y in self._bounds],
                    },
                    stream,
                    allow_unicode=True,
                    sort_keys=False)
            if not quiet:
                self.get_logger().info(f'Saved boundary to {self._bounds_file}')
            return True
        except Exception as exc:  # noqa: BLE001 - keep node alive on IO errors
            self.get_logger().warning(f'Cannot save boundary: {exc}')
            return False

    def _load_file(self, quiet: bool) -> bool:
        if not self._bounds_file or not os.path.isfile(self._bounds_file):
            return False
        try:
            with open(self._bounds_file, encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
            points = data.get('points', [])
            loaded = [
                (float(p[0]), float(p[1]))
                for p in points
                if isinstance(p, (list, tuple)) and len(p) >= 2]
            if len(loaded) < 3:
                self.get_logger().warning(
                    f'Ignoring boundary file with {len(loaded)} vertices: '
                    f'{self._bounds_file}')
                return False
            self._bounds = loaded
            self._publish_bounds()
            self._publish_bounds_markers()
            if not quiet:
                self.get_logger().info(
                    f'Loaded {len(self._bounds)} vertices from '
                    f'{self._bounds_file}')
            return True
        except Exception as exc:  # noqa: BLE001 - keep node alive on IO errors
            self.get_logger().warning(f'Cannot load boundary: {exc}')
            return False

    # ------------------------------------------------------------- publishers

    def _publish_bounds(self) -> None:
        msg = PolygonStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in self._bounds]
        self._bounds_pub.publish(msg)

    def _publish_bounds_markers(self) -> None:
        arr = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        stamp = self.get_clock().now().to_msg()
        for index, (x, y) in enumerate(self._bounds):
            sphere = Marker()
            sphere.header.frame_id = self._frame
            sphere.header.stamp = stamp
            sphere.ns = 'bounds_points'
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20
            sphere.color.r, sphere.color.g = 1.0, 0.35
            sphere.color.b, sphere.color.a = 0.0, 1.0
            arr.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self._frame
            label.header.stamp = stamp
            label.ns = 'bounds_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(index + 1)
            arr.markers.append(label)

        if len(self._bounds) >= 2:
            line = Marker()
            line.header.frame_id = self._frame
            line.header.stamp = stamp
            line.ns = 'bounds_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.08
            line.color.r, line.color.g = 0.1, 1.0
            line.color.b, line.color.a = 0.2, 0.9
            line.points = [
                Point(x=float(x), y=float(y), z=0.05)
                for x, y in self._bounds]
            if len(self._bounds) >= 3:
                line.points.append(line.points[0])
            arr.markers.append(line)

        self._bounds_marker_pub.publish(arr)

    def _publish_guide_markers(self) -> None:
        arr = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        if self._guide is not None:
            stamp = self.get_clock().now().to_msg()
            guide = Marker()
            guide.header.frame_id = self._frame
            guide.header.stamp = stamp
            guide.ns = 'guide_arrow'
            guide.id = 0
            guide.type = Marker.ARROW
            guide.action = Marker.ADD
            guide.pose = self._guide.pose
            guide.scale.x = 2.0
            guide.scale.y = 0.22
            guide.scale.z = 0.22
            guide.color.r, guide.color.g = 1.0, 0.9
            guide.color.b, guide.color.a = 0.0, 1.0
            arr.markers.append(guide)

            point = Marker()
            point.header.frame_id = self._frame
            point.header.stamp = stamp
            point.ns = 'guide_point'
            point.id = 1
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position = self._guide.pose.position
            point.pose.position.z = 0.25
            point.pose.orientation.w = 1.0
            point.scale.x = point.scale.y = point.scale.z = 0.25
            point.color.r, point.color.g = 1.0, 0.9
            point.color.b, point.color.a = 0.0, 1.0
            arr.markers.append(point)

            label = Marker()
            label.header.frame_id = self._frame
            label.header.stamp = stamp
            label.ns = 'guide_label'
            label.id = 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = self._guide.pose.position
            label.pose.position.z = 0.7
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = 'GUIDE'
            arr.markers.append(label)

        self._guide_marker_pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = ExplorationUI()
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
