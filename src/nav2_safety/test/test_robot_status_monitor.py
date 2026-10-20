"""Regression checks for event status, feedback liveness and stuck detection."""
import json
from types import MethodType, SimpleNamespace as NS

import pytest
from rclpy.time import Time

from nav2_safety.robot_status_monitor import RobotStatusMonitor


@pytest.fixture
def monitor():
    clock = NS(time=Time(nanoseconds=0))
    m = NS(
        _nav_active=False, _nav_status=None, _nav_goal_id=None,
        _nav_status_time=None, _last_goal_distance=None, _last_recoveries=0,
        _number_of_recoveries=0, _distance_remaining=None,
        _last_progress=clock.time, _cmd_active_since=clock.time,
        _cmd_smoothed_active=True, _cmd_smoothed_time=clock.time,
        _checked_zero_since=clock.time, _recovery_cmd_time=None,
        _recovery_active={}, _recovery_status_time={},
        _explore_status=None, _odom_time=clock.time, _scan_time=clock.time,
        _estop=False, _autonomy_blocked=False, _tracking_ok=True,
        _deadman_stopping=False, _recovery_blocked=False,
        _recovery_guard_active=False, _recovery_guard_active_time=None,
        _watchdog_detail='', _recovery_guard_detail='',
        _recovery_guard_time=None, _start_time=clock.time,
        _nav_status_timeout=2.5, _cmd_timeout=0.5, _stuck_timeout=12.0,
        _progress_epsilon=0.05, _collision_stop_time=0.5,
        _odom_timeout=0.5, _scan_timeout=1.0, _startup_grace=8.0,
        _battery_msg=None, _battery_time=None, _battery_timeout=2.0,
        _battery_low_soc=20.0, _battery_critical_soc=10.0,
        _battery_voltage_full=29.4, _battery_voltage_empty=21.0,
        _battery_charge_current_sign=-1.0,
        _battery_charge_current_threshold=0.2,
        _publish_marker=False, get_clock=lambda: NS(now=lambda: clock.time),
        _status_pub=NS(publish=lambda msg: setattr(m, 'payload', json.loads(msg.data))),
    )
    for name in ('_now', '_age', '_on_nav_status', '_on_feedback', '_on_explore',
                 '_on_recovery_active', '_active_recovery', '_choose_state',
                 '_battery_payload', '_tick'):
        setattr(m, name, MethodType(getattr(RobotStatusMonitor, name), m))

    def advance(seconds):
        clock.time = Time(nanoseconds=int(seconds * 1e9))
        m._odom_time = m._scan_time = m._cmd_smoothed_time = clock.time

    m.advance = advance
    m.advance(0)
    return m


def status(value=2, identity=1):
    return NS(status_list=[NS(
        status=value, goal_info=NS(stamp=NS(sec=0, nanosec=0),
                                  goal_id=NS(uuid=[identity] * 16)))])


def feedback(identity=1, distance=10.0):
    return NS(goal_id=NS(uuid=[identity] * 16),
              feedback=NS(number_of_recoveries=0, distance_remaining=distance))


def test_long_navigation_stays_live_on_feedback_without_status_heartbeat(monitor):
    m = monitor
    m._on_nav_status(status())
    m._on_explore(NS(status='exploration_started'))
    for second in range(1, 91):
        m.advance(second)
        m._on_feedback(feedback(distance=100.0 - second))
        m._tick()
        assert m.payload['state'] == 'NAVIGATING'
        assert m.payload['nav_active']
        assert m.payload['explore_status'] == 'exploration_started'


def test_stationary_navigation_reports_stuck_even_when_status_is_old(monitor):
    m = monitor
    m._on_nav_status(status())
    for second in range(1, 16):
        m.advance(second)
        m._on_feedback(feedback())
        m._tick()
    assert m.payload['state'] == 'STUCK'
    assert m.payload['collision_stop']
    assert m.payload['stuck_for'] >= 12


def test_recovery_heartbeat_keeps_long_action_fresh(monitor):
    m = monitor
    m._on_nav_status(status())
    m._recovery_active['smart_recovery'] = True
    m._recovery_status_time['smart_recovery'] = m._now()
    # Action status stays latched for the whole 20 s action; only the guard
    # heartbeat updates. Recovery must not age out after 2.5 s.
    m.advance(10)
    m._on_recovery_active(NS(data=True))
    m.advance(10.5)
    m._tick()
    assert m.payload['state'] == 'RECOVERING'
    assert m.payload['recovery_action'] == 'smart_recovery'


def test_late_feedback_does_not_refresh_a_different_goal(monitor):
    m = monitor
    m._on_nav_status(status(identity=2))
    m.advance(5)
    m._on_feedback(feedback(identity=1))
    m._tick()
    assert not m.payload['nav_active']


def test_terminal_goal_cannot_be_revived_by_late_feedback(monitor):
    m = monitor
    m._on_nav_status(status())
    m._on_nav_status(status(4))
    m.advance(5)
    m._on_feedback(feedback())
    m._tick()
    assert m.payload['state'] == 'IDLE'
    assert not m.payload['nav_active']


@pytest.mark.parametrize('event,expected', [
    ('exploration_started', 'EXPLORING'),
    ('exploration_in_progress', 'EXPLORING'),
    ('exploration_complete', 'EXPLORE_DONE'),
    ('exploration_paused', 'IDLE'),
    ('exploration_blocked', 'STUCK'),
])
def test_latched_exploration_events_persist_until_next_transition(monitor, event, expected):
    m = monitor
    m._on_explore(NS(status=event))
    m.advance(600)
    m._tick()
    assert m.payload['state'] == expected
    assert m.payload['explore_status'] == event
    m._on_explore(NS(status='exploration_paused'))
    m._tick()
    assert m.payload['state'] == 'IDLE'


def test_battery_payload_prefers_bms_soc(monitor):
    m = monitor
    m._battery_msg = NS(
        battery_voltage=26.0, battery_soc=55, battery_soh=98,
        battery_current=-1.2, battery_temperature=30.0)
    m._battery_time = m.get_clock().now()
    m._tick()
    b = m.payload['battery']
    assert b['online'] is True
    assert b['source'] == 'bms'
    assert b['soc'] == 55
    assert b['charging'] is True
    assert b['low'] is False
    assert b['critical'] is False


def test_battery_payload_falls_back_to_voltage(monitor):
    m = monitor
    m._battery_msg = NS(
        battery_voltage=25.2, battery_soc=0, battery_soh=0,
        battery_current=0.0, battery_temperature=0.0)
    m._battery_time = m.get_clock().now()
    m._tick()
    b = m.payload['battery']
    assert b['online'] is True
    assert b['source'] == 'voltage'
    assert b['soc'] == 50
    assert b['charging'] is False


def test_battery_low_and_critical_flags(monitor):
    m = monitor
    m._battery_msg = NS(
        battery_voltage=23.0, battery_soc=8, battery_soh=90,
        battery_current=2.0, battery_temperature=28.0)
    m._battery_time = m.get_clock().now()
    m._tick()
    b = m.payload['battery']
    assert b['low'] is True
    assert b['critical'] is True
