"""Online tangent estimator (paper Sec. 3.3, Eq. 4; Lutscher et al. 2010).

Low-pass filtered end-effector velocity, a speed gate, sign alignment and a
window of unit samples whose mean resultant length is the confidence c.
"""

from collections import deque

import numpy as np


class ConstraintEstimator:

    def __init__(self, beta=0.05, v_min=0.01, window=50, d0=None):
        if not 0.0 < beta <= 1.0:
            raise ValueError('beta must be in (0, 1]')
        self.beta = beta
        self.v_min = v_min
        self.window = int(window)
        self._d0 = None if d0 is None else _unit(np.asarray(d0, dtype=float))
        self.reset()

    def reset(self):
        self.v_bar = np.zeros(3)
        self.samples = deque(maxlen=self.window)
        self.d_hat = None if self._d0 is None else self._d0.copy()
        self.c = 0.0

    def update(self, p_dot):
        """Feed one velocity measurement; returns (d_hat or None, c)."""
        p_dot = np.asarray(p_dot, dtype=float)
        self.v_bar = (1.0 - self.beta) * self.v_bar + self.beta * p_dot
        speed = np.linalg.norm(self.v_bar)
        if speed > self.v_min:
            m = self.v_bar / speed
            if self.d_hat is not None and m @ self.d_hat < 0.0:
                m = -m
            self.samples.append(m)
            s = np.sum(self.samples, axis=0)
            s_norm = np.linalg.norm(s)
            if s_norm > 1e-9:
                self.d_hat = s / s_norm
            # Divide by N, not len(samples): confidence ramps up as the
            # window fills, which gives the start-up phase for free.
            self.c = s_norm / self.window
        return self.d_hat, self.c


def _unit(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError('zero vector has no direction')
    return v / n
