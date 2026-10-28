"""DualSense axes to Cartesian velocity command (paper block 3).

u = g * v_max * R * phi(a): left stick gives the horizontal direction, right
stick vertical, R2 depression g in [0, 1] is throttle and deadman.
"""

import numpy as np


def deadzone(a, dz=0.1):
    """Radial dead-zone on the stick vector, rescaled so |phi(a)| <= 1."""
    a = np.asarray(a, dtype=float)
    n = np.linalg.norm(a)
    if n <= dz:
        return np.zeros_like(a)
    return a / n * min((n - dz) / (1.0 - dz), 1.0)


def user_velocity(a, g, v_max=0.1, R=None, dz=0.1):
    """a = (x, y, z) stick axes in the viewing frame, g = throttle."""
    phi = deadzone(a, dz)
    if R is None:
        R = np.eye(3)
    return float(np.clip(g, 0.0, 1.0)) * v_max * (np.asarray(R) @ phi)
