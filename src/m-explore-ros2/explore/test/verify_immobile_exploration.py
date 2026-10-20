"""Isolated test: stationary navigation failures must pause, never finish exploration.

Source install/setup.bash; use ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1.
Only starts explore_lite with a synthetic map/TF and a fake NavigateToPose server.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from explore_lite_msgs.msg import ExploreStatus
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '174'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
node = rclpy.create_node('immobile_exploration_fixture')
tf = TransformBroadcaster(node)
map_pub = node.create_publisher(OccupancyGrid, '/test/global', 10)
resume_pub = node.create_publisher(Bool, '/explore/resume', 10)
statuses = []
latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(ExploreStatus, '/explore/status', lambda m: statuses.append(m.status), latched)
state = dict(origin_blocked=False, goals=0)


def execute(handle):
    state['goals'] += 1
    handle.abort()
    return NavigateToPose.Result()


server = ActionServer(node, NavigateToPose, '/navigate_to_pose', execute)


def publish():
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in [('map', 'odom'), ('odom', 'base_footprint')]:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.rotation.w = 1.0
        if child == 'base_footprint':
            msg.transform.translation.x = msg.transform.translation.y = 0.05
        transforms.append(msg)
    tf.sendTransform(transforms)
    grid = OccupancyGrid()
    grid.header.frame_id, grid.header.stamp = 'map', stamp
    grid.info.width = grid.info.height = 60
    grid.info.resolution = 0.1
    grid.info.origin.position.x = grid.info.origin.position.y = -3.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0 if 18 <= x < 42 and 18 <= y < 42 else -1
                 for y in range(60) for x in range(60)]
    if state['origin_blocked']:
        grid.data[30*60+30] = 100
    map_pub.publish(grid)


node.create_timer(0.05, publish)
logs = Path(tempfile.mkdtemp(prefix='immobile_exploration_'))
process = None


def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if process is not None:
            assert process.poll() is None, f'explore exited: {logs}'


def stop_process():
    global process
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4)
    process = None


try:
    for occupied in (False, True):
        state.update(origin_blocked=occupied, goals=0)
        statuses.clear()
        spin_for(0.2)
        with (logs / f'occupied_{occupied}.log').open('w') as log:
            args = ['ros2', 'run', 'explore_lite', 'explore', '--ros-args']
            parameters = dict(costmap_topic='/test/global', costmap_updates_topic='/test/updates',
                              robot_base_frame='base_footprint', planner_frequency=5.0,
                              gain_scale=1.0, potential_scale=5.0, min_frontier_size=0.1,
                              no_valid_frontier_grace=0.5, startup_grace=0.0,
                              goal_snap_radius=1.5, visualize=False)
            for key, value in parameters.items():
                args += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            deadline = time.monotonic() + 10
            while 'exploration_blocked' not in statuses and time.monotonic() < deadline:
                spin_for(0.05)
            assert 'exploration_blocked' in statuses, f'No blocked status: {logs}'
            assert 'exploration_complete' not in statuses, 'Immobility falsely reported complete'
            count = state['goals']
            spin_for(0.7)
            assert state['goals'] == count == 1, 'Kept issuing goals while immobilized'
            if not occupied:
                resume_pub.publish(Bool(data=True))
                spin_for(1.5)
                assert state['goals'] == 2, 'Explicit resume could not retry unproven frontier'
                assert 'exploration_complete' not in statuses
            print(f'origin_occupied={occupied}: paused without false completion; goals={state["goals"]}', flush=True)
            stop_process()
    print(f'All immobile exploration checks passed. Logs: {logs}', flush=True)
finally:
    stop_process()
    server.destroy()
    node.destroy_node()
    rclpy.shutdown()
