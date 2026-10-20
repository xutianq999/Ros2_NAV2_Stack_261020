"""Escape directions, coupled motion, slowing, ownership and failure handling."""
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from nav2_safety.recovery_activity import RecoveryActivity
from nav2_safety.recovery_guard import RecoveryGuard
from nav2_safety.recovery_motion import blocking, limit_motion


def points(values):
    return np.array(values, dtype=float).reshape(-1, 2)


def limit(values, vx=0.0, wz=0.0):
    return limit_motion(points(values), vx, wz)


def wall(x, y=0.0):
    return [(x, y - 0.01), (x, y + 0.01)]


@pytest.mark.parametrize('vx', [-0.1, 0.1])
def test_wall_on_opposite_side_allows_full_speed(vx):
    v, w, factor = limit(wall(-math.copysign(0.375, vx)), vx)
    assert (v, w, factor) == (vx, 0.0, 1.0)


@pytest.mark.parametrize('vx', [-0.1, 0.1])
def test_wall_in_motion_direction_blocks(vx):
    assert limit(wall(math.copysign(0.37, vx)), vx) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('vx', [-0.1, 0.1])
@pytest.mark.parametrize('gap', [0.005, 0.01, 0.03])
def test_side_wall_allows_tangential_translation(vx, gap):
    pts = [(-0.2, 0.285 + gap), (0.2, 0.285 + gap)]
    assert limit(pts, vx) == (vx, 0.0, 1.0)


def test_old_17cm_buffer_no_longer_blocks_13cm_clearance():
    assert limit(wall(-0.48), -0.1) == (-0.1, 0.0, 1.0)


def test_limited_room_slows_before_stopping():
    v, w, factor = limit(wall(-0.40), -0.1)
    assert -0.1 < v <= -0.02 and w == 0 and 0 < factor < 1


def test_slowing_preserves_arc_curvature():
    # Rear wall leaves enough room for a slower arc but not the requested arc.
    v, w, factor = limit(wall(-0.42), -0.1, 0.1)
    assert 0 < factor < 1 and v < 0 and w > 0
    assert w / v == pytest.approx(-1.0)


def test_caps_preserve_arc_curvature():
    v, w, factor = limit([], 1.0, 3.0)
    assert v <= 0.15 and w <= 0.8 and factor < 1
    assert w / v == pytest.approx(3.0)


def test_arc_checks_corner_outside_straight_lane_and_slows():
    pts = [(-0.38, -0.30), (-0.375, -0.30)]
    assert limit(pts, -0.1)[0] < 0.0
    assert blocking(points(pts), -0.1, 0.15, 0.5, 0.345, 0.285, 0.025) >= 2
    v, w, factor = limit(pts, -0.1, 0.15)
    assert 0 < factor < 1
    assert blocking(points(pts), v, w, 0.5, 0.345, 0.285, 0.025) < 2


def test_close_corner_rotation_away_allowed_toward_blocked():
    pts = [(0.345, 0.29), (0.34, 0.292), (0.335, 0.295)]
    assert limit(pts, wz=-0.5)[1] < 0
    assert limit(pts, wz=0.5) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('vx,wz', [(-0.1, 0), (0.1, 0), (0, -0.5), (0, 0.5), (-0.1, 0.1)])
def test_points_inside_footprint_block_every_motion(vx, wz):
    assert limit([(0.05, 0.05), (0.08, 0.02)], vx, wz) == (0, 0, 0)


def test_logged_side_obstacles_allow_straight_escape():
    pts = [(-0.40, 0.36), (-0.41, 0.35), (-0.41, 0.34), (-0.43, 0.32),
           (-0.44, 0.32), (-0.43, 0.33), (-0.42, 0.35)]
    assert limit(pts, -0.1) == (-0.1, 0, 1)


class Pub:
    def __init__(self): self.last = None
    def publish(self, msg): self.last = msg


def tick(vx, wz, pts=(), scan_age=0.0, cmd_age=0.0, active=0, vy=0.0):
    from geometry_msgs.msg import Twist
    s = NS(_probe_owners=lambda: None, _now=lambda: 100.0,
           _activity=NS(count=lambda now: active), _active_pub=Pub(),
           _pub=Pub(), _blocked_pub=Pub(), _status=lambda *args: None,
           _cmd_wall=100.0-cmd_age, _scan_wall=100.0-scan_age,
           _input_timeout=0.3, _scan_timeout=0.5, _policy={},
           _pts=None if pts is None else points(pts), _cmd=Twist())
    s._cmd.linear.x, s._cmd.angular.z, s._cmd.linear.y = float(vx), float(wz), float(vy)
    RecoveryGuard._tick(s)
    return s


@pytest.mark.parametrize('pts,age', [(None, 0), ([], 2.0)])
def test_missing_or_stale_scan_stops(pts, age):
    s = tick(-0.1, 0.2, pts, scan_age=age)
    assert s._pub.last.linear.x == s._pub.last.angular.z == 0


def test_active_command_gap_holds_zero_priority_but_idle_releases():
    assert tick(-0.1, 0, cmd_age=1, active=0)._pub.last is None
    s = tick(-0.1, 0, cmd_age=1, active=1)
    assert s._pub.last.linear.x == s._pub.last.angular.z == 0


def test_simultaneous_actions_stop():
    assert tick(-0.1, 0, active=2)._pub.last.linear.x == 0


@pytest.mark.parametrize('vx,wz,vy', [(float('nan'), 0, 0), (0, float('inf'), 0), (0.1, 0, 0.1)])
def test_unsupported_commands_stop(vx, wz, vy):
    s = tick(vx, wz, vy=vy)
    assert s._pub.last.linear.x == s._pub.last.angular.z == 0
    assert s._blocked_pub.last.data


def status(value=2):
    return NS(status=value, goal_info=NS(goal_id=NS(uuid=[1]*16)))


def test_terminal_status_releases_immediately():
    a = RecoveryActivity()
    a.update('smart', [status()], 10)
    assert a.count(11) == 1
    a.update('smart', [status(4)], 11)
    assert a.count(11) == 0


class _ResultFuture:
    def __init__(self, status): self._status = status
    def cancelled(self): return False
    def result(self): return NS(status=self._status)


def test_unknown_result_keeps_ownership():
    a = RecoveryActivity()
    a.update('smart', [status()], 10)
    key = bytes([1] * 16)
    s = NS(_result_requests={('smart', key): object()}, _activity=a)
    RecoveryGuard._on_result(s, 'smart', key, _ResultFuture(0))
    assert a.count(10.2) == 1


def test_completed_cache_is_bounded():
    a = RecoveryActivity()
    for i in range(300):
        a.finish('smart', bytes([i % 256, i // 256]))
    assert len(a.completed) <= 256


def test_result_confirmation_releases_without_terminal_status():
    a = RecoveryActivity()
    a.update('smart', [status()], 10)
    a.finish('smart', bytes([1]*16))
    assert a.count(10.2) == 0
    a.update('smart', [status()], 10.3)  # delayed active status must not resurrect it
    assert a.count(10.3) == 0


def test_server_disappearance_releases_ownership():
    a = RecoveryActivity()
    a.update('smart', [status()], 10)
    a.server_gone('smart')
    assert a.count(11) == 0


def test_canceling_retains_ownership_until_confirmed():
    a = RecoveryActivity()
    a.update('smart', [status(3)], 10)
    assert a.count(11) == 1
    a.finish('smart', bytes([1]*16))
    assert a.count(11.1) == 0


def test_status_cannot_extend_final_lease_forever():
    a = RecoveryActivity()
    a.update('smart', [status()], 10)
    a.update('smart', [status()], 30)
    assert a.count(36) == 0


def test_planner_and_guard_share_clearance_configuration():
    root = Path(__file__).resolve().parents[2]
    guard = yaml.safe_load((root/'nav2_safety/config/recovery_guard.yaml').read_text())['recovery_guard']['ros__parameters']
    planner = yaml.safe_load((root/'me_nav2_bringup/config/nav2_params.yaml').read_text())['behavior_server']['ros__parameters']['smart_recovery']
    for key in ('half_length', 'half_width', 'min_clearance', 'min_points', 'scan_timeout', 'scan_topic', 'min_linear', 'min_angular', 'lookahead_time'):
        assert guard[key] == planner[key], key


def test_owner_probe_uses_resolved_topic_and_releases_disappeared_server():
    import time
    a = RecoveryActivity()
    now = time.monotonic()
    a.update('smart', [status()], now-1)
    seen_topics = []
    def publishers(topic):
        seen_topics.append(topic)
        return 0
    s = NS(_last_owner_probe=0.0, _activity=a, _status_seen={'smart': now-1},
           _status_topics={'smart': '/remapped/smart'}, count_publishers=publishers,
           _result_requests={}, _request_result=lambda *args: None)
    RecoveryGuard._probe_owners(s)
    assert seen_topics == ['/remapped/smart'] and a.count(now) == 0


def test_result_callback_clears_owner_without_terminal_topic_event():
    a = RecoveryActivity()
    a.update('smart', [status()], 1.0)
    key = bytes([1]*16)
    s = NS(_activity=a, _result_requests={('smart', key): object()})
    future = NS(cancelled=lambda: False, result=lambda: NS(status=4))
    RecoveryGuard._on_result(s, 'smart', key, future)
    assert a.count(1.1) == 0 and not s._result_requests


def test_result_discovery_is_retried_when_initial_service_was_missing():
    import time
    a = RecoveryActivity()
    now = time.monotonic()
    a.update('smart', [status()], now)
    requests = []
    s = NS(_last_owner_probe=0, _activity=a, _status_seen={'smart': now},
           _status_topics={'smart': '/smart/status'}, count_publishers=lambda topic: 1,
           _result_requests={}, _request_result=lambda name, key: requests.append((name, key)))
    RecoveryGuard._probe_owners(s)
    assert requests == [('smart', bytes([1]*16))]


def test_stale_scan_does_not_refresh_valid_observation():
    from sensor_msgs.msg import LaserScan
    scan = LaserScan()
    scan.angle_increment, scan.range_min, scan.range_max = 0.1, 0.01, 10.0
    scan.header.stamp.sec = 90
    scan.ranges = [1.0, 1.0]
    s = NS(_now=lambda: 100.0, _scan_timeout=0.5, _scan_wall=99.0)
    RecoveryGuard._on_scan(s, scan)
    assert s._scan_wall == 99.0
