"""Force split and DualSense haptic mapping (paper Eqs. 3 and 8).

Sign convention: F is the force the robot applies to the handle, expressed in
the base frame (i.e. the negated wrist F/T reading after gravity removal).
"""

import numpy as np


def force_split(F, d_hat):
    """Returns (F_par, F_perp) w.r.t. the tangent d_hat (Eq. 3)."""
    F = np.asarray(F, dtype=float)
    if d_hat is None:
        return 0.0, F.copy()
    d = np.asarray(d_hat, dtype=float)
    F_par = float(d @ F)
    return F_par, F - F_par * d


def _sat01(x):
    return float(np.clip(x, 0.0, 1.0))


def rumble(F_perp, u_perp, v_max, F0=2.0, F1=20.0, kappa_u=0.5):
    """Rumble intensity r in [0, 1] (Eq. 8, left)."""
    f_term = (np.linalg.norm(F_perp) - F0) / (F1 - F0)
    u_term = kappa_u * np.linalg.norm(u_perp) / v_max
    return _sat01(max(f_term, 0.0) + u_term)


def trigger_resistance(F_par, Fpar0=2.0, Fpar1=15.0):
    """Adaptive-trigger resistance rho in [0, 1] (Eq. 8, right)."""
    return _sat01((abs(F_par) - Fpar0) / (Fpar1 - Fpar0))
