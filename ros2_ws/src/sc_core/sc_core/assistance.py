"""Confidence-weighted guidance virtual fixture (paper Sec. 3.4, Eq. 6)."""

import numpy as np

DIRECT = 'direct'
ASSISTED = 'assisted'


def k_perp(c, k_min=0.05, c0=0.6, c1=0.9):
    """Off-axis gain: 1 while c <= c0, falls linearly to k_min at c >= c1."""
    s = np.clip((c - c0) / (c1 - c0), 0.0, 1.0)
    return 1.0 - (1.0 - k_min) * s


def projector(d_hat):
    d = np.asarray(d_hat, dtype=float)
    return np.outer(d, d)


def assist(u, d_hat, c, mode=ASSISTED, k_min=0.05, c0=0.6, c1=0.9):
    """Returns (v_cmd, u_perp). u_perp is the part the fixture rejects."""
    u = np.asarray(u, dtype=float)
    if d_hat is None:
        return u.copy(), np.zeros(3)
    P = projector(d_hat)
    u_perp = u - P @ u
    if mode == DIRECT:
        return u.copy(), u_perp
    if mode != ASSISTED:
        raise ValueError('unknown mode: %s' % mode)
    return P @ u + k_perp(c, k_min, c0, c1) * u_perp, u_perp


def integrate_setpoint(x_d, v_cmd, x, d_hat, c, dt, perp_leak=0.0, c1=0.9):
    """x_d <- x_d + dt * v_cmd (Alg. 1, line 15).

    With perp_leak > 0 and a confident estimate (c >= c1), the off-axis part
    of the tracking error x_d - x also decays at rate perp_leak, so lateral
    error built up before the fixture engaged does not persist as force.
    """
    x_d = np.asarray(x_d, dtype=float) + dt * np.asarray(v_cmd, dtype=float)
    if perp_leak > 0.0 and d_hat is not None and c >= c1:
        e = x_d - np.asarray(x, dtype=float)
        e_perp = e - projector(d_hat) @ e
        x_d = x_d - min(perp_leak * dt, 1.0) * e_perp
    return x_d
