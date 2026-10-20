"""Temporal confirmation must reject passing returns and retain static walls."""
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from grid_mapper.grid_mapper_node import GridMapper
from grid_mapper.static_hits import StaticHitFilter


def observe(f, stamp, cells=((10, 20),)):
    return f.update([p[0] for p in cells], [p[1] for p in cells], stamp)


def test_dense_duplicate_returns_in_one_frame_do_not_confirm():
    f = StaticHitFilter()
    assert not observe(f, 0, ((10, 20),) * 1000).any()
    assert not observe(f, 0, ((10, 20),) * 1000).any()
    assert not observe(f, 0.1).any()


def test_static_cell_needs_observation_duration_and_multiple_frames():
    f = StaticHitFilter()
    for stamp in np.arange(0, 2, 0.1):
        assert not observe(f, float(stamp)).any()
    assert observe(f, 2.0).all()


def test_scanning_gap_restarts_confirmation():
    f = StaticHitFilter()
    for stamp in np.arange(0, 1.5, 0.1):
        observe(f, float(stamp))
    assert not observe(f, 2.1).any()
    assert f._cells[(10, 20)][0] == 2.1


def test_free_space_invalidates_confirmation_immediately():
    f = StaticHitFilter()
    for stamp in np.arange(0, 2.1, 0.1):
        observe(f, float(stamp))
    f.clear([10], [20])
    assert not observe(f, 2.2).any()


def test_clock_rewind_starts_a_new_observation_episode():
    f = StaticHitFilter()
    for stamp in np.arange(0, 2.1, 0.1):
        observe(f, float(stamp))
    assert not observe(f, 0.1).any()


def test_old_candidates_are_pruned_without_forgetting_other_cells():
    f = StaticHitFilter()
    observe(f, 0, ((1, 1),))
    observe(f, 0.4, ((2, 2),))
    observe(f, 0.6, ((2, 2),))
    assert (1, 1) not in f._cells
    assert (2, 2) in f._cells


@pytest.mark.parametrize('duration,gap', [(-1, 0.5), (float('nan'), 1),
                                          (2, 0), (2, float('inf'))])
def test_invalid_parameters_are_rejected(duration, gap):
    with pytest.raises(ValueError):
        StaticHitFilter(duration, gap)


def mapper(duration=2.0, ray_stride=1):
    m = NS(_res=0.05, _min_range=0.3, _max_clear_range=10.0,
           _ray_stride=ray_stride, _z_min=-0.2, _z_max=1.2,
           _hit_inc=4, _miss_inc=-1, _lo_min=-24, _lo_max=32,
           _occ_thresh=8, _free_thresh=-3,
           _min_ix=-50, _max_ix=50, _min_iy=-50, _max_iy=50,
           _logodds=np.zeros((101, 101), dtype=np.int16), _used=None,
           _static_hits=StaticHitFilter(duration, cell_size=3),
           get_logger=lambda: NS(info=lambda msg: None))
    for name in ('_integrate', '_apply', '_expand_to'):
        setattr(m, name, MethodType(getattr(GridMapper, name), m))
    return m


def integrate(m, stamp, pts, origin=(0, 0)):
    m._integrate(origin, np.asarray(pts, dtype=np.float32).reshape(-1, 3), stamp)


def score(m, x, y):
    return m._logodds[round(y / m._res) - m._min_iy,
                      round(x / m._res) - m._min_ix]


def test_passing_person_never_becomes_a_saved_map_obstacle():
    m = mapper()
    # Dense 0.5 m body moving 0.5 m/s, visible for 4 seconds.
    for frame in range(40):
        t = frame / 10
        pts = [(2, y, 0.5) for y in np.arange(t * 0.5, t * 0.5 + 0.5, 0.02)] * 20
        integrate(m, t, pts)
    assert not (m._logodds >= m._occ_thresh).any()


def test_walls_are_confirmed_and_do_not_expire_when_not_visible():
    m = mapper()
    for frame in range(31):
        integrate(m, frame / 10, [(1, 0, 0.5)] * 100)
    assert score(m, 1, 0) >= m._occ_thresh
    integrate(m, 100, [(0, 2, -0.3)])
    assert score(m, 1, 0) >= m._occ_thresh


def test_person_standing_then_leaving_is_cleared_by_new_free_observations():
    m = mapper()
    for frame in range(41):
        integrate(m, frame / 10, [(1, 0, 0.5)] * 100)
    assert score(m, 1, 0) >= m._occ_thresh
    for frame in range(41, 81):
        integrate(m, frame / 10, [(2, 0, -0.3)])
    assert score(m, 1, 0) <= m._free_thresh
    # Reappearing must start confirmation again.
    integrate(m, 8.1, [(1, 0, 0.5)] * 100)
    assert score(m, 1, 0) <= m._free_thresh


@pytest.mark.parametrize('ray_stride', [1, 2])
def test_unconfirmed_body_does_not_clear_through_it_or_clear_existing_wall(ray_stride):
    m = mapper(ray_stride=ray_stride)
    integrate(m, 0, [(1, 0, 0.5)] * 100)
    assert score(m, 1, 0) == 0
    assert score(m, 1.5, 0) == 0
    iy, ix = -m._min_iy, round(1 / m._res) - m._min_ix
    m._logodds[iy, ix] = 32
    integrate(m, 0.1, [(1, 0, 0.5), (2, 0, -0.3)])
    assert score(m, 1, 0) == 32


def test_confirmation_survives_map_expansion_with_absolute_cell_coordinates():
    m = mapper()
    for frame in range(31):
        integrate(m, frame / 10, [(1, 0, 0.5), (10, 10, 0.5)])
    assert score(m, 1, 0) >= m._occ_thresh
    assert score(m, 10, 10) >= m._occ_thresh


def test_confirmation_can_be_disabled_for_legacy_mapping():
    m = mapper(duration=0)
    integrate(m, 0, [(1, 0, 0.5)] * 100)
    assert score(m, 1, 0) == 32


def test_wall_jitter_across_fine_cells_shares_confirmation():
    f = StaticHitFilter(cell_size=3)
    for frame in range(21):
        result = observe(f, frame / 10, ((frame % 3, 0),))
    assert result.all()


def test_free_rays_next_to_observed_wall_do_not_reset_its_neighborhood():
    f = StaticHitFilter(cell_size=3)
    for frame in range(21):
        f.clear([1], [0], protected_ix=[0], protected_iy=[0])
        result = observe(f, frame / 10, ((0, 0),))
    assert result.all()
    f.clear([1], [0])
    assert not observe(f, 2.1, ((0, 0),)).any()
