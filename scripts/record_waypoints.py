#!/usr/bin/env python3
"""Record named map-frame waypoints from RViz 2D Goal Pose clicks."""

import math
import os
import threading

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        default_file = os.path.join(os.getcwd(), 'waypoints.yaml')
        self.declare_parameter('output_file', default_file)
        self.declare_parameter('record_enabled', True)
        self.output_file = self.get_parameter('output_file').value
        self.record_enabled = self.get_parameter('record_enabled').value
        self.points = self._load_points()
        self.lock = threading.Lock()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', latched_qos)
        if self.record_enabled:
            self.goal_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.publish_markers()
        mode = 'recording clicks' if self.record_enabled else 'read-only display'
        self.get_logger().info(f'Ready ({mode}); waypoint file: {self.output_file}')

    def _load_points(self):
        if not os.path.isfile(self.output_file):
            return []
        try:
            with open(self.output_file, encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
            return data.get('points', [])
        except Exception as exc:  # keep recorder usable if file is malformed
            self.get_logger().warning(f'Cannot read existing waypoints: {exc}')
            return []

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def goal_callback(self, msg):
        with self.lock:
            index = len(self.points) + 1
            point = {
                'id': index,
                'name': f'point_{index}',
                'x': round(float(msg.pose.position.x), 4),
                'y': round(float(msg.pose.position.y), 4),
                'yaw': round(self._yaw_from_quaternion(msg.pose.orientation), 4),
            }
            self.points.append(point)
            os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    {'frame': msg.header.frame_id or 'map', 'points': self.points},
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )
            self.publish_markers()
            self.get_logger().info(
                f"Saved {point['name']}: x={point['x']:.4f}, "
                f"y={point['y']:.4f}, yaw={point['yaw']:.4f}")

    def publish_markers(self):
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        for point in self.points:
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.ns = 'waypoints'
            sphere.id = point['id'] * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point['x']
            sphere.pose.position.y = point['y']
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.2, 0.1, 1.0
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = 'map'
            label.ns = 'waypoint_labels'
            label.id = point['id'] * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = point['x']
            label.pose.position.y = point['y']
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(point['id'])
            markers.markers.append(label)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
