#!/usr/bin/env python3
"""Summarise the Phase 3 pilot trials (host or container, needs numpy + matplotlib).

    python3 scripts/analyze_pilot.py

Reads results/phase3/trials/*.json|csv and writes, in results/phase3/:
summary.json, pilot_table.tex (included by the report), pilot_outcomes.png
and pilot_traces.png.
"""

import glob
import json
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'phase3')
PI = 3.14159
YAW_LABEL = {'3.14159': 0.0, '2.74159': -0.4, '3.54159': 0.4}


def load():
    trials = []
    for path in sorted(glob.glob(os.path.join(ROOT, 'trials', '*.json'))):
        name = os.path.basename(path)[:-5]
        yaw, h, rep = re.match(r'yaw([\d.]+)_h([\d.]+)_r(\d+)', name).groups()
        m = json.load(open(path))
        m.update(name=name, tilt=YAW_LABEL[yaw], h=float(h), rep=int(rep))
        trials.append(m)
    return trials


def table(trials):
    """Per (tilt, heading) statistics, and one LaTeX row per heading."""
    rows, tex = [], []
    tilts = sorted({t['tilt'] for t in trials})
    for h in sorted({t['h'] for t in trials}):
        cells = []
        for tilt in tilts:
            g = [t for t in trials if t['tilt'] == tilt and t['h'] == h]
            op, pk = [t['final_opening_m'] for t in g], [t['peak_F_perp_N'] for t in g]
            s = sum(t['status'] == 'success' for t in g)
            rows.append(dict(tilt=tilt, h=h, n=len(g), success=s, opening=float(np.mean(op)),
                             opening_std=float(np.std(op)), peak=float(np.mean(pk)),
                             statuses=sorted({t['status'] for t in g})))
            cells.append('$%.3f \\pm %.3f$' % (np.mean(op), np.std(op)) if len(g) > 1 else '$%.3f$' % op[0])
        g = [t for t in trials if t['h'] == h]
        tex.append('%g & %d/%d & %s & $%.1f$ \\\\' % (
            h, sum(t['status'] == 'success' for t in g), len(g), ' & '.join(cells),
            max(t['peak_F_perp_N'] for t in g)))
    with open(os.path.join(ROOT, 'pilot_table.tex'), 'w') as f:
        f.write('\n'.join(tex) + '\n')
    return rows


def plots(trials):
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for tilt, c, mk in ((0.0, 'C0', 'o'), (-0.4, 'C1', 's'), (0.4, 'C2', '^')):
        g = [t for t in trials if t['tilt'] == tilt]
        x = np.array([t['h'] for t in g])
        ax[0].scatter(x * (1 + 0.15 * tilt) + 0.02 * tilt, [t['final_opening_m'] for t in g], c=c, marker=mk,
                      label='axis tilt %+.1f rad' % tilt, zorder=3)
        ax[1].scatter(x * (1 + 0.15 * tilt) + 0.02 * tilt, [t['peak_F_perp_N'] for t in g], c=c, marker=mk, zorder=3)
    ax[0].axhline(0.25, color='k', ls='--', lw=1, label='target 0.25 m')
    ax[1].axhline(30.0, color='k', ls='--', lw=1, label='handle loss 30 N')
    ax[0].set_ylabel('opening reached [m]')
    ax[1].set_ylabel(r'peak $\|F_\perp\|$ [N]')
    for a in ax:
        a.set_xscale('symlog', linthresh=0.5)
        a.set_xticks([0, 0.5, 1, 2, 3, 5, 10, 20])
        a.set_xticklabels(['0', '0.5', '1', '2', '3', '5', '10', '20'])
        a.set_xlabel('operator heading error [deg]')
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'pilot_outcomes.png'), dpi=160)

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    for h, c in ((0, 'C0'), (0.5, 'C2'), (1, 'C1'), (3, 'C3')):
        path = os.path.join(ROOT, 'trials', 'yaw3.14159_h%g_r1.csv' % h)
        if os.path.exists(path):
            d = np.loadtxt(path, delimiter=',', skiprows=1)
            ax[0].plot(d[:, 0], d[:, 3], c, label='%g deg' % h)
            ax[1].plot(d[:, 0], d[:, 1], c, label='%g deg' % h)
    ax[0].axhline(30.0, color='k', ls='--', lw=1)
    ax[1].axhline(0.25, color='k', ls='--', lw=1)
    ax[0].set_ylabel(r'$\|F_\perp\|$, 50 ms filter [N]')
    ax[1].set_ylabel('opening [m]')
    for a in ax:
        a.set_xlabel('time [s]')
        a.grid(alpha=0.3)
        a.legend(fontsize=8, title='heading error')
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'pilot_traces.png'), dpi=160)


def main():
    trials = load()
    rows = table(trials)
    plots(trials)
    summary = dict(n_trials=len(trials),
                   outcomes={s: sum(t['status'] == s for t in trials)
                             for s in sorted({t['status'] for t in trials})},
                   rows=rows)
    with open(os.path.join(ROOT, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print(summary['outcomes'], 'of', len(trials))
    for r in rows:
        print('tilt %+.1f h %5g: %d/%d success, opening %.3f +- %.3f, peak %.1f %s'
              % (r['tilt'], r['h'], r['success'], r['n'], r['opening'], r['opening_std'],
                 r['peak'], r['statuses']))


if __name__ == '__main__':
    main()
