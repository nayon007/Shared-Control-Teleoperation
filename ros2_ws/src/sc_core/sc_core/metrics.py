"""Trial metrics (paper block 11). Off-axis force is recomputed with the
ground-truth tangent so a biased estimate cannot hide it."""

import numpy as np


def compute_metrics(log, slip_force, target_opening):
    t = np.asarray(log['t'])
    F_perp_true = np.linalg.norm(np.asarray(log['F_perp_true']), axis=1)
    opening = np.asarray(log['opening'])
    reached = np.nonzero(opening >= target_opening)[0]
    lost = np.nonzero(F_perp_true > slip_force)[0]
    return {
        'opening_time_s': float(t[reached[0]]) if reached.size else float('nan'),
        'final_opening_m': float(opening[-1]),
        'mean_F_perp_N': float(F_perp_true.mean()),
        'peak_F_perp_N': float(F_perp_true.max()),
        'handle_lost': bool(lost.size),
        'handle_loss_time_s': float(t[lost[0]]) if lost.size else float('nan'),
    }
