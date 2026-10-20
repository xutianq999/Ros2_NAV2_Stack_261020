"""Manual isolated ROS integration test; never starts hardware or publishes /cmd_vel.

Run after sourcing install/setup.bash with ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1.
Exercises real behavior plugins, recovery_guard and twist_mux with synthetic data.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped, Point32, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from ament_index_python.packages import get_package_prefix
from nav2_msgs.action import BackUp, Spin, Wait
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as PathMsg
from action_msgs.msg import GoalStatus, GoalStatusArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '173', 'Use isolated ROS domain 173'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = Path(__file__).resolve().parents[3]
assert Path(get_package_prefix('me_nav2_bringup')).resolve() == root / 'install/me_nav2_bringup', 'Wrong workspace overlay'
logs = Path(tempfile.mkdtemp(prefix='recovery_actions_'))
processes, log_files = [], []


def start(package, executable, config, remaps=()):
    log = (logs / f'{executable}.log').open('w')
    log_files.append(log)
    args = ['ros2', 'run', package, executable, '--ros-args', '--params-file', str(config)]
    for remap in remaps:
        args += ['-r', remap]
    processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True))


start('nav2_behaviors', 'behavior_server', root / 'src/me_nav2_bringup/config/nav2_params.yaml',
      ['cmd_vel:=/cmd_vel_recovery'])
start('nav2_safety', 'recovery_guard', root / 'src/nav2_safety/config/recovery_guard.yaml',
      ['/smart_recovery/_action/status:=/recovery_test/smart_status',
       '/drive_on_heading/_action/status:=/recovery_test/orphan_status',
       '/drive_on_heading/_action/get_result:=/recovery_test/missing_result'])
start('twist_mux', 'twist_mux', root / 'src/nav2_safety/config/twist_mux.yaml',
      ['cmd_vel_out:=/recovery_test/muxed'])
start('nav2_safety', 'deadman', root / 'src/nav2_safety/config/deadman.yaml',
      ['/cmd_vel:=/recovery_test/chassis', '/cmd_vel_muxed:=/recovery_test/muxed'])
rclpy.init()
node = rclpy.create_node('recovery_test_fixture')
tf = TransformBroadcaster(node)
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
grid_pub = node.create_publisher(Costmap, '/local_costmap/costmap_raw', latched)
global_pub = node.create_publisher(Costmap, '/global_costmap/costmap_raw', latched)
fp_pub = node.create_publisher(PolygonStamped, '/local_costmap/published_footprint', latched)
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
nav_pub = node.create_publisher(Twist, '/cmd_vel_nav_checked', 10)
estop_pub = node.create_publisher(Bool, '/safety/estop_active', 10)
watchdog_pub = node.create_publisher(Bool, '/safety/autonomy_blocked', 10)
status_relay = node.create_publisher(GoalStatusArray, '/recovery_test/smart_status', latched)


def relay_active_only(msg):
    # Deliberately drop every smart-recovery terminal status. GetResult must release ownership.
    active = [s for s in msg.status_list if s.status in (1, 2, 3)]
    if active:
        out = GoalStatusArray()
        out.status_list = active
        status_relay.publish(out)


node.create_subscription(GoalStatusArray, '/smart_recovery/_action/status', relay_active_only, latched)
pose = [0.0, 0.0, 0.0]
state = dict(moving=False, cmd=Twist(), cmd_at=0.0, active=False, mux=None,
             path=None, paths=[], wall=False, block_straight=False, saw_arc=False,
             obstacles=[], global_block_at=None)


def checked(msg):
    state.update(cmd=msg, cmd_at=time.monotonic())


def path_received(msg):
    state['path'] = msg
    if msg.poses:
        state['paths'].append(msg)


node.create_subscription(Twist, '/recovery_test/chassis', checked, 10)
node.create_subscription(Twist, '/recovery_test/muxed', lambda m: state.update(mux=m), 10)
node.create_subscription(Bool, '/safety/recovery_active', lambda m: state.update(active=m.data), 10)
node.create_subscription(PathMsg, '/recovery_path', path_received, 10)
last_tick = time.monotonic()


def publish():
    global last_tick
    now = time.monotonic()
    dt, last_tick = min(now - last_tick, 0.1), now
    if state['moving'] and now - state['cmd_at'] < 0.3:
        v, w = state['cmd'].linear.x, state['cmd'].angular.z
        if abs(w) > 0.01 and abs(v) > 0.01:
            state.update(block_straight=False, saw_arc=True)
        if state['block_straight']:
            v = w = 0.0
        yaw = pose[2]
        if abs(w) < 1e-8:
            pose[0] += v * dt * math.cos(yaw)
            pose[1] += v * dt * math.sin(yaw)
        else:
            pose[0] += v / w * (math.sin(yaw + w * dt) - math.sin(yaw))
            pose[1] -= v / w * (math.cos(yaw + w * dt) - math.cos(yaw))
        pose[2] += w * dt
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in [('map', 'odom'), ('odom', 'base_footprint')]:
        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id, tr.child_frame_id = parent, child
        tr.transform.rotation.w = 1.0
        if child == 'base_footprint':
            tr.transform.translation.x, tr.transform.translation.y = pose[:2]
            tr.transform.rotation.z = math.sin(pose[2] / 2)
            tr.transform.rotation.w = math.cos(pose[2] / 2)
        else:
            tr.transform.translation.x = 1.0  # map differs from costmap/odom
        transforms.append(tr)
    tf.sendTransform(transforms)
    grid = Costmap()
    grid.header.stamp, grid.header.frame_id = stamp, 'odom'
    grid.metadata.resolution = 0.05
    grid.metadata.size_x = grid.metadata.size_y = 120
    grid.metadata.origin.position.x = grid.metadata.origin.position.y = -3.0
    grid.metadata.origin.orientation.w = 1.0
    grid.data = ([254 if (i % 120) * 0.05 - 3.0 < -0.55 else 0 for i in range(14400)]
                 if state['wall'] else [0] * 14400)
    grid_pub.publish(grid)
    global_grid = Costmap()
    global_grid.header.stamp, global_grid.header.frame_id = stamp, 'map'
    global_grid.metadata.resolution = 0.05
    global_grid.metadata.size_x = global_grid.metadata.size_y = 120
    global_grid.metadata.origin.position.x = global_grid.metadata.origin.position.y = -3.0
    global_grid.metadata.origin.orientation.w = 1.0
    global_grid.data = [0] * 14400
    if state['global_block_at'] is not None:
        bx, by = state['global_block_at']
        mx = int((bx + 1.0 - global_grid.metadata.origin.position.x) / 0.05)
        my = int((by - global_grid.metadata.origin.position.y) / 0.05)
        # A point exactly on a cell boundary may truncate to either side.
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ix, iy = mx + dx, my + dy
                if 0 <= ix < 120 and 0 <= iy < 120:
                    global_grid.data[iy * 120 + ix] = 254
    global_pub.publish(global_grid)
    fp = PolygonStamped()
    fp.header = grid.header
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for x, y in [(0.345, 0.285), (0.345, -0.285), (-0.345, -0.285), (-0.345, 0.285)]:
        fp.polygon.points.append(Point32(x=float(pose[0] + c*x - s*y),
                                        y=float(pose[1] + s*x + c*y)))
    fp_pub.publish(fp)
    scan = LaserScan()
    scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2 * math.pi / 359
    scan.range_min, scan.range_max = 0.01, 10.0
    scan.ranges = [2.0] * 360
    for world_x, world_y in state['obstacles']:
        dx, dy = world_x - pose[0], world_y - pose[1]
        x, y = c * dx + s * dy, -s * dx + c * dy
        angle = math.atan2(y, x)
        index = max(0, min(359, round((angle + math.pi) / scan.angle_increment)))
        scan.ranges[index] = math.hypot(x, y)
    scan_pub.publish(scan)
    cmd = Twist()
    cmd.linear.x = 0.12  # Deliberately keep nav active to test mux ownership.
    nav_pub.publish(cmd)


node.create_timer(0.05, publish)


def spin_for(seconds):
    until_time = time.monotonic() + seconds
    while time.monotonic() < until_time:
        rclpy.spin_once(node, timeout_sec=0.02)
        assert all(p.poll() is None for p in processes), f'Node exited: {logs}'


def until(future, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        spin_for(0.02)
    assert future.done(), f'Timed out: {logs}'
    return future.result()


def send(name, typ, target=0.5, allowance=30):
    client = ActionClient(node, typ, name)
    assert client.wait_for_server(timeout_sec=5.0), name
    goal = typ.Goal()
    if typ is BackUp:
        goal.target.x, goal.speed = target, 0.1
        goal.time_allowance.sec = allowance
    elif typ is Spin:
        goal.target_yaw = target
        goal.time_allowance.sec = allowance
    else:
        goal.time.sec = 2
    handle = until(client.send_goal_async(goal))
    assert handle.accepted
    return client, handle


def run_action(name, typ, target, expected, moving=False, timeout=15.0):
    state['moving'] = moving
    client, handle = send(name, typ, target)
    start_time = time.monotonic()
    result = until(handle.get_result_async(), timeout=timeout)
    elapsed = time.monotonic() - start_time
    state['moving'] = False
    print(f'{name}: status={result.status}, elapsed={elapsed:.2f}s, pose={pose}', flush=True)
    assert result.status == expected, f'{name}: see {logs}'
    if expected == 6:
        assert 1.8 < elapsed < (12.0 if name == 'smart_recovery' else 4.0)
    spin_for(0.25)
    if name == 'smart_recovery':
        assert state['path'] is not None and not state['path'].poses, 'Path not cleared'
    client.destroy()


try:
    lifecycle = node.create_client(ChangeState, '/behavior_server/change_state')
    assert lifecycle.wait_for_service(timeout_sec=8.0)
    for transition in [1, 3]:
        req = ChangeState.Request()
        req.transition.id = transition
        assert until(lifecycle.call_async(req)).success
    spin_for(1.0)
    run_action('backup', BackUp, 0.5, 6)
    run_action('spin', Spin, 1.57, 6)
    run_action('smart_recovery', BackUp, 0.5, 6)
    assert state['paths'], 'No recovery visualization published'
    for path in state['paths']:
        assert path.header.frame_id == 'odom'
        first, last = path.poses[0].pose, path.poses[-1].pose
        a = 2 * math.atan2(first.orientation.z, first.orientation.w)
        b = 2 * math.atan2(last.orientation.z, last.orientation.w)
        assert abs(math.atan2(math.sin(b-a), math.cos(b-a))) <= 0.521
    pose[:] = [0.0, 0.0, 0.0]
    spin_for(0.2)
    run_action('smart_recovery', BackUp, 0.5, 4, moving=True)
    assert pose[0] < -0.04 and abs(pose[1]) < 0.02
    # Simulate straight-motion stall, then let the selected arc move the robot.
    pose[:] = [0.0, 0.0, 0.0]
    state.update(block_straight=True, saw_arc=False)
    spin_for(0.2)
    run_action('smart_recovery', BackUp, 0.5, 4, moving=True)
    assert state['saw_arc'] and abs(pose[1]) > 0.0004
    # Live rear wall: old 17 cm buffer blocked immediately; new policy plans short safe steps.
    pose[:] = [0.0, 0.0, 0.0]
    state['obstacles'] = [(-0.48, -0.05), (-0.48, 0.05)]
    spin_for(0.3)
    before = len(state['paths'])
    run_action('smart_recovery', BackUp, 0.5, 4, moving=True)
    assert pose[0] < -0.04 and 0.48 + pose[0] - 0.345 > 0.02
    first_path = state['paths'][before]
    assert -0.07 < first_path.poses[-1].pose.position.x < -0.03
    state['obstacles'] = []
    # Both outer locks still suppress recovery at the final output.
    pose[:] = [0.0, 0.0, 0.0]
    spin_for(0.3)
    client, handle = send('backup', BackUp)
    spin_for(0.2)
    assert state['cmd'].linear.x < 0
    for publisher in (estop_pub, watchdog_pub):
        publisher.publish(Bool(data=True))
        spin_for(0.45)
        assert state['cmd'].linear.x == state['cmd'].angular.z == 0.0
        publisher.publish(Bool(data=False))
        spin_for(0.15)
        assert state['cmd'].linear.x < 0
    until(handle.cancel_goal_async())
    assert until(handle.get_result_async()).status == 5
    client.destroy()
    # Cancel while moving: action must stop and clear its path.
    pose[:] = [0.0, 0.0, 0.0]
    spin_for(0.2)
    client, handle = send('smart_recovery', BackUp)
    spin_for(0.3)
    assert state['active']
    until(handle.cancel_goal_async())
    assert until(handle.get_result_async()).status == 5
    spin_for(0.25)
    assert not state['path'].poses
    assert state['cmd'].linear.x == state['cmd'].angular.z == 0.0
    client.destroy()
    # Wait sends no ongoing velocity; ownership must still block the live nav stream.
    spin_for(1.0)
    assert abs(state['mux'].linear.x - 0.12) < 1e-6, 'Idle recovery did not release nav'
    client, handle = send('wait', Wait)
    spin_for(0.8)
    assert state['active']
    assert state['mux'].linear.x == state['mux'].angular.z == 0.0, 'Nav leaked during wait'
    assert until(handle.get_result_async()).status == 4
    spin_for(1.1)
    assert not state['active'] and abs(state['mux'].linear.x - 0.12) < 1e-6
    client.destroy()
    # An owner disappears without results or a terminal status: release promptly.
    orphan = node.create_publisher(GoalStatusArray, '/recovery_test/orphan_status', latched)
    spin_for(0.3)
    msg = GoalStatusArray()
    item = GoalStatus()
    item.status = 2
    item.goal_info.goal_id.uuid = [171] * 16
    msg.status_list = [item]
    orphan.publish(msg)
    spin_for(0.3)
    assert state['active'] and state['mux'].linear.x == 0.0
    node.destroy_publisher(orphan)
    spin_for(2.0)
    assert not state['active'] and abs(state['mux'].linear.x - 0.12) < 1e-6
    # Rear wall shortens the selected primitive; cancel before synthetic motion.
    pose[:] = [0.0, 0.0, 0.0]
    state['wall'] = True
    spin_for(0.3)
    client, handle = send('smart_recovery', BackUp)
    spin_for(0.3)
    path = state['path']
    assert path.poses and -0.19 < path.poses[-1].pose.position.x < -0.03
    until(handle.cancel_goal_async())
    assert until(handle.get_result_async()).status == 5
    client.destroy()
    # Global costmap marks the start lethal while the local map/scan are clear:
    # SmartRecovery must search for a nearby traversable cell and leave it.
    pose[:] = [0.0, 0.0, 0.0]
    state['wall'] = False
    state['obstacles'] = []
    state['global_block_at'] = (0.0, 0.0)
    spin_for(0.3)
    before = len(state['paths'])
    run_action('smart_recovery', BackUp, 0.5, 4, moving=True, timeout=35.0)
    assert math.hypot(pose[0], pose[1]) > 0.03, f'free-space escape did not move: {pose}'
    assert len(state['paths']) > before, 'No free-space path published'
    assert len(state['paths'][before].poses) > 1
    state['global_block_at'] = None
    print(f'All recovery action / mux / cancellation checks passed. Logs: {logs}', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
    for log in log_files:
        log.close()
