"""Confirm map obstacles across sensor frames without expiring established walls.

This is temporal persistence, not semantic person detection. A stationary person
can pass confirmation; new free-space observations must still clear their cells.
"""
import math

import numpy as np


class StaticHitFilter:
    """Track candidate cells in absolute grid coordinates, across map growth."""

    def __init__(self, confirmation_time=2.0, max_gap=0.5, cell_size=1):
        if (not math.isfinite(confirmation_time) or confirmation_time < 0
                or not math.isfinite(max_gap) or max_gap <= 0):
            raise ValueError('Static hit confirmation must be >= 0 and gap > 0')
        if not isinstance(cell_size, int) or cell_size < 1:
            raise ValueError('Confirmation cell size must be a positive integer')
        self.confirmation_time = confirmation_time
        self.max_gap = max_gap
        self.cell_size = cell_size
        self._cells = {}
        self._stamp = None
        self._last_prune = None

    def update(self, ix, iy, stamp):
        """Return a confirmation mask for unique hit cells in this sensor frame."""
        if not math.isfinite(stamp):
            raise ValueError('Cloud timestamp must be finite')
        if self._stamp is not None and stamp < self._stamp:
            self._cells.clear()
            self._last_prune = None
        self._stamp = stamp
        if self._last_prune is None or stamp - self._last_prune >= self.max_gap:
            self._cells = {k: v for k, v in self._cells.items()
                           if stamp - v[1] <= self.max_gap}
            self._last_prune = stamp
        confirmed = np.zeros(len(ix), dtype=bool)
        for index, (x, y) in enumerate(zip(ix, iy)):
            key = (int(x) // self.cell_size, int(y) // self.cell_size)
            previous = self._cells.get(key)
            if previous is None or stamp - previous[1] > self.max_gap:
                state = [stamp, stamp, 1]
            else:
                state = previous
                if stamp > state[1]:
                    state[2] += 1
                state[1] = stamp
            self._cells[key] = state
            confirmed[index] = (state[2] >= 3
                                and stamp - state[0] >= self.confirmation_time)
        return confirmed

    def clear(self, ix, iy, protected_ix=(), protected_iy=()):
        """Free-space evidence invalidates an obstacle's pending confirmation."""
        if self._cells:
            protected = {(int(x) // self.cell_size, int(y) // self.cell_size)
                         for x, y in zip(protected_ix, protected_iy)}
            for x, y in zip(ix, iy):
                key = (int(x) // self.cell_size, int(y) // self.cell_size)
                if key not in protected:
                    self._cells.pop(key, None)
