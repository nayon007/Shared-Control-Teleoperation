#!/usr/bin/env python3
"""Step 7 analysis: direct vs assisted trials with the impedance controller.

    python3 scripts/analyze_step7.py

Reads results/step7/trials/*.json and *.csv (written by run_experiments.sh)
and writes into results/step7/:
  summary.md          table of every condition
  summary_table.tex   the same for the report
  peak_force.png      peak off-axis force against heading error
  traces.png          force, opening and confidence over time at 10 and 20 deg
"""

import glob
import json
import os
import re
from collections import defaultdict

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'step7')
NAME = re.compile(r'(drawer|door)_yaw([\d.]+)_(direct|assisted|assistednoleak)_h([\d.]+)_r(\d+)')
LABEL = {'direct': 'direct', 'assisted': 'assisted', 'assistednoleak': 'assisted, no leak'}
COLOR = {'direct': 'C3', 'assisted': 'C0', 'assistednoleak': 'C2'}


def load():
    rows = []
    for f in sorted(glob.glob(os.path.join(ROOT, 'trials', '*.json'))):
        m = NAME.match(os.path.basename(f)[:-5])
        if not m:
            continue
        d = json.load(open(f))
        d.update(kind=m[1], yaw=float(m[2]), cond=m[3], heading=float(m[4]), rep=int(m[5]),
                 csv=f[:-5] + '.csv')
        rows.append(d)
    return rows


def group(rows):
    g = defaultdict(list)
    for r in rows:
        g[(r['kind'], r['yaw'], r['cond'], r['heading'])].append(r)
    return g


def table(g):
    md = ['| mechanism | axis yaw | control | heading [deg] | trials | success | opening [m] | '
          'mean off-axis [N] | peak off-axis [N] |', '|' + '---|' * 9]
    tex = []
    for key in sorted(g, key=lambda k: (k[0] != 'drawer', abs(k[1] - 3.14159) > 1e-3, k[1],
                                         k[3], list(LABEL).index(k[2]))):
        kind, yaw, cond, h = key
        rs = g[key]
        ok = sum(r['status'] == 'success' for r in rs)
        op = np.mean([r['final_opening_m'] for r in rs])
        mf = np.mean([r['mean_F_perp_N'] for r in rs])
        pf = np.mean([r['peak_F_perp_N'] for r in rs])
        md.append('| %s | %.2f | %s | %g | %d | %d | %.3f | %.1f | %.1f |'
                  % (kind, yaw, LABEL[cond], h, len(rs), ok, op, mf, pf))
        tex.append('%s & %.2f & %s & %g & %d/%d & %.3f & %.1f & %.1f \\\\'
                   % (kind, yaw, LABEL[cond], h, ok, len(rs), op, mf, pf))
    return '\n'.join(md) + '\n', '\n'.join(tex) + '\n'


def plots(g, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    for ax, (kind, yaw, title) in zip(axes, (('drawer', 3.14159, 'Drawer (axis towards the robot)'),
                                            ('door', 3.14159, 'Door (radius 0.6 m)'))):
        for cond in LABEL:
            hs = sorted({k[3] for k in g if k[:3] == (kind, yaw, cond)})
            if not hs:
                continue
            pk = [np.mean([r['peak_F_perp_N'] for r in g[(kind, yaw, cond, h)]]) for h in hs]
            lost = [any(r['handle_lost'] for r in g[(kind, yaw, cond, h)]) for h in hs]
            ax.plot(hs, pk, 'o-', color=COLOR[cond], label=LABEL[cond])
            ax.plot([h for h, x in zip(hs, lost) if x], [p for p, x in zip(pk, lost) if x],
                    'x', color='k', ms=10)
        ax.axhline(30, ls='--', color='k', lw=1)
        ax.text(0.3, 31, 'handle lost (30 N)', fontsize=8)
        ax.set_xlabel('operator heading error [deg]')
        ax.set_ylabel('peak off-axis force [N]')
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'peak_force.png'), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(3, 2, figsize=(10, 7.5), sharex='col')
    for j, h in enumerate((10.0, 20.0)):
        for cond in ('direct', 'assisted'):
            rs = g.get(('drawer', 3.14159, cond, h))
            if not rs or not os.path.exists(rs[0]['csv']):
                continue
            d = np.genfromtxt(rs[0]['csv'], delimiter=',', names=True)
            ax[0, j].plot(d['t'], d['F_perp'], color=COLOR[cond], label=LABEL[cond])
            ax[1, j].plot(d['t'], d['opening'], color=COLOR[cond])
            ax[2, j].plot(d['t'], d['c'], color=COLOR[cond])
        ax[0, j].axhline(30, ls='--', color='k', lw=1)
        ax[0, j].set_title('Drawer, heading error %g deg' % h)
        ax[0, j].set_ylabel('off-axis force [N]')
        ax[1, j].set_ylabel('opening [m]')
        ax[2, j].set_ylabel('confidence c')
        ax[2, j].set_xlabel('time [s]')
        ax[0, j].legend(fontsize=8)
        for a in ax[:, j]:
            a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'traces.png'), dpi=150)
    plt.close(fig)


def main():
    rows = load()
    if not rows:
        raise SystemExit('no trials in ' + ROOT)
    g = group(rows)
    md, tex = table(g)
    open(os.path.join(ROOT, 'summary.md'), 'w').write(md)
    open(os.path.join(ROOT, 'summary_table.tex'), 'w').write(tex)
    plots(g, rows)
    n_ok = defaultdict(lambda: [0, 0])
    for r in rows:
        n_ok[r['cond']][0] += r['status'] == 'success'
        n_ok[r['cond']][1] += 1
    print(md)
    for c, (ok, n) in n_ok.items():
        print('%-16s %d / %d succeeded' % (LABEL[c], ok, n))


if __name__ == '__main__':
    main()
