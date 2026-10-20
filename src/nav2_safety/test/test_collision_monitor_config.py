"""Regression guard: collision checking must stay direction-aware.

The old StopZone/SlowZone were fixed rectangles that ignored motion direction.
A wall beside the robot stayed inside the rectangle, so forward, reverse and
rotation commands were all zeroed while the robot was merely driving parallel
to it. Keep only the footprint sweep, which simulates the commanded motion.
"""
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'collision_monitor.yaml'


def params():
    return yaml.safe_load(CONFIG.read_text())['collision_monitor']['ros__parameters']


def test_only_direction_aware_polygon_is_enabled():
    p = params()
    assert p['polygons'] == ['FootprintApproach']
    assert 'StopZone' not in p
    assert 'SlowZone' not in p


def test_approach_uses_real_footprint_and_simulates_motion():
    approach = params()['FootprintApproach']
    assert approach['action_type'] == 'approach'
    assert approach['footprint_topic'] == '/local_costmap/published_footprint'
    assert approach['time_before_collision'] > 0.0
    assert approach['simulation_time_step'] > 0.0
    assert approach['enabled'] is True


def test_observation_source_present():
    p = params()
    assert p['observation_sources'] == ['scan']
    assert p['scan']['topic'] == '/scan'
    assert p['scan']['enabled'] is True
