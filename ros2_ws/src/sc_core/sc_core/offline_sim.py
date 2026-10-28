"""Offline closed-loop simulation of the shared-control loop (paper Alg. 1).

Pure numpy, no ROS or Gazebo. The end-effector is welded to the handle, so it
moves only along the mechanism's path; a Cartesian impedance law
F = Kp (x_d - x) - Kd x_dot (the translational part of Eq. 7) produces the
force the robot applies to the handle. The along-path part drives the
mechanism, the off-axis part is what the paper wants to suppress.

Physics runs at 1 kHz, the shared-control loop at 100 Hz (the ROS rate).

    python3 -m sc_core.offline_sim            # table of direct vs assisted
    python3 -m sc_core.offline_sim --plot out.png
"""

import argparse

import numpy as np

from .assistance import assist, integrate_setpoint, ASSISTED, DIRECT
from .estimator import ConstraintEstimator
from .haptics import force_split, rumble, trigger_resistance
from .mechanisms import Prismatic, Revolute
from .metrics import compute_metrics
from .params import Params


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def run_trial(mech, mode, prm=None, heading_err_deg=20.0, target=0.25,
              t_max=15.0, Kp=400.0, Kd=40.0, mass=3.0, damping=5.0,
              coulomb=3.0, vel_noise=0.002, force_noise=0.3, seed=0):
    """Simulated operator pushes along a fixed direction, the initial tangent
    rotated by heading_err_deg, at full throttle until the handle has
    travelled `target` metres along the path."""
    prm = prm or Params()
    rng = np.random.default_rng(seed)
    dt_phys, ctrl_every = 1e-3, 10
    u_dir = _rot_z(np.deg2rad(heading_err_deg)) @ mech.tangent(0.0)

    est = ConstraintEstimator(prm.beta, prm.v_min, prm.window)
    d_hat, c = None, 0.0
    s, s_dot = 0.0, 0.0                       # arc length along the path
    q_of_s = (lambda s_: s_ / mech.radius) if isinstance(mech, Revolute) else (lambda s_: s_)
    x_d = mech.position(0.0).copy()
    v_cmd = np.zeros(3)
    F = np.zeros(3)
    log = {k: [] for k in ('t', 'opening', 'F_perp_true', 'F_perp_est', 'F_par',
                           'u', 'v_cmd', 'd_hat', 'c', 'r', 'rho', 'angle_err_deg')}

    for k in range(int(t_max / dt_phys)):
        q = q_of_s(s)
        d_star = mech.tangent(q)
        x = mech.position(q)
        x_dot = d_star * s_dot

        if k % ctrl_every == 0:
            t = k * dt_phys
            opening = mech.opening(q)
            g = 1.0 if opening < target else 0.0
            u = g * prm.v_max * u_dir
            meas_v = x_dot + rng.normal(0.0, vel_noise, 3)
            meas_F = F + rng.normal(0.0, force_noise, 3)
            d_hat, c = est.update(meas_v)
            v_cmd, u_perp = assist(u, d_hat, c, mode, prm.k_min, prm.c0, prm.c1)
            F_par, F_perp = force_split(meas_F, d_hat)
            _, F_perp_true = force_split(F, d_star)
            r = rumble(F_perp, u_perp, prm.v_max, prm.F0, prm.F1, prm.kappa_u)
            rho = trigger_resistance(F_par, prm.Fpar0, prm.Fpar1)

            ang = np.nan if d_hat is None else np.degrees(
                np.arccos(np.clip(abs(d_hat @ d_star), 0.0, 1.0)))
            for key, val in (('t', t), ('opening', opening),
                             ('F_perp_true', F_perp_true), ('F_perp_est', F_perp),
                             ('F_par', F_par), ('u', u), ('v_cmd', v_cmd),
                             ('d_hat', np.full(3, np.nan) if d_hat is None else d_hat),
                             ('c', c), ('r', r), ('rho', rho), ('angle_err_deg', ang)):
                log[key].append(val)
            if np.linalg.norm(F_perp_true) > prm.slip_force:
                break                          # handle lost: trial ends

        x_d = integrate_setpoint(x_d, v_cmd, x, d_hat if mode == ASSISTED else None,
                                 c, dt_phys, prm.perp_leak, prm.c1)
        F = Kp * (x_d - x) - Kd * x_dot
        f_drive = d_star @ F - damping * s_dot - coulomb * np.tanh(s_dot / 1e-3)
        s_dot += dt_phys * f_drive / mass
        s += dt_phys * s_dot
        s = max(s, 0.0)                        # closed stop
        if s == 0.0:
            s_dot = max(s_dot, 0.0)

    log = {k: np.asarray(v) for k, v in log.items()}
    return log, compute_metrics(log, prm.slip_force, target)


def scenarios(n_axes=5, seed=1):
    """Drawers with randomised horizontal axis and doors of varying radius."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n_axes):
        yaw = rng.uniform(-np.pi, np.pi)
        out.append(('drawer#%d' % i,
                    Prismatic([0.5, 0.0, 0.4], [np.cos(yaw), np.sin(yaw), 0.0])))
    for i, radius in enumerate((0.4, 0.6, 0.8)):
        out.append(('door#%d r=%.1f' % (i, radius),
                    Revolute([0.5, -radius, 0.4], radius, theta0=np.pi / 2)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--heading-err', type=float, default=20.0,
                    help='operator heading error [deg]')
    ap.add_argument('--perp-leak', type=float, default=0.0,
                    help='off-axis set-point relaxation [1/s], 0 = paper baseline')
    ap.add_argument('--plot', help='save a figure of one drawer and one door trial')
    args = ap.parse_args(argv)
    prm = Params(perp_leak=args.perp_leak)

    hdr = '%-14s %-9s %9s %9s %10s %10s %6s' % (
        'scenario', 'mode', 'time[s]', 'open[m]', 'meanFp[N]', 'peakFp[N]', 'lost')
    print(hdr)
    print('-' * len(hdr))
    for name, mech in scenarios():
        for mode in (DIRECT, ASSISTED):
            _, m = run_trial(mech, mode, prm, heading_err_deg=args.heading_err)
            print('%-14s %-9s %9.2f %9.3f %10.2f %10.2f %6s' % (
                name, mode, m['opening_time_s'], m['final_opening_m'],
                m['mean_F_perp_N'], m['peak_F_perp_N'], 'YES' if m['handle_lost'] else 'no'))

    if args.plot:
        _plot(args.plot, args.heading_err, prm)


def _plot(path, heading_err, prm):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    cases = [s for s in scenarios() if s[0] in ('drawer#0', 'door#1 r=0.6')]
    fig, ax = plt.subplots(3, len(cases), figsize=(11, 8), sharex='col')
    for j, (name, mech) in enumerate(cases):
        for mode, style in ((DIRECT, 'C3'), (ASSISTED, 'C0')):
            log, _ = run_trial(mech, mode, prm, heading_err_deg=heading_err)
            ax[0, j].plot(log['t'], np.linalg.norm(log['F_perp_true'], axis=1), style, label=mode)
            ax[1, j].plot(log['t'], log['opening'], style)
            ax[2, j].plot(log['t'], log['c'], style)
            ax[2, j].plot(log['t'], log['angle_err_deg'] / 90.0, style, ls='--')
        ax[0, j].set_title(name)
        ax[0, j].set_ylabel('|F_perp| true [N]')
        ax[1, j].set_ylabel('opening [m]')
        ax[2, j].set_ylabel('c (solid), angle err/90deg (dashed)')
        ax[2, j].set_xlabel('t [s]')
        ax[0, j].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print('saved', path)


if __name__ == '__main__':
    main()
