#!/usr/bin/env python3
"""Friction study analysis (run_friction_study.sh) on the host.

    python3 scripts/analyze_friction.py

Writes results/friction/summary.md and friction.png: success, peak off-axis
force, pulling force and opening time against the drawer's joint friction,
for direct and assisted control at each heading error.
"""

import glob
import json
import os
import re
from collections import defaultdict

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'friction')
NAME = re.compile(r'f([\d.]+)_(direct|assisted)_h([\d.]+)')
COLOR = {'direct': 'C3', 'assisted': 'C0'}
STYLE = {0.0: ':', 10.0: '--', 20.0: '-'}


def main():
    rows = []
    for f in sorted(glob.glob(os.path.join(ROOT, 'trials', '*.json'))):
        m = NAME.match(os.path.basename(f)[:-5])
        if not m:
            continue
        d = json.load(open(f))
        csv = f[:-5] + '.csv'
        log = np.genfromtxt(csv, delimiter=',', names=True) if os.path.exists(csv) else None
        d.update(friction=float(m[1]), mode=m[2], heading=float(m[3]),
                 peak_F_par=float(np.abs(log['F_par']).max()) if log is not None else np.nan,
                 mean_F_par=float(np.abs(log['F_par'][log['u'] > 0]).mean())
                 if log is not None else np.nan)
        rows.append(d)
    if not rows:
        raise SystemExit('no trials in ' + ROOT)

    md = ['| friction [N] | heading [deg] | mode | result | opening [m] | time to 0.25 m [s] | '
          'peak off-axis [N] | mean pulling force [N] |', '|' + '---|' * 8]
    for r in sorted(rows, key=lambda r: (r['friction'], r['heading'], r['mode'] != 'direct')):
        md.append('| %g | %g | %s | %s | %.3f | %s | %.1f | %.1f |' % (
            r['friction'], r['heading'], r['mode'], r['status'], r['final_opening_m'],
            '-' if np.isnan(r['opening_time_s']) else '%.2f' % r['opening_time_s'],
            r['peak_F_perp_N'], r['mean_F_par']))
    ok = defaultdict(lambda: [0, 0])
    for r in rows:
        ok[r['mode']][0] += r['status'] == 'success'
        ok[r['mode']][1] += 1
    md += ['', '%s' % ', '.join('%s %d/%d succeeded' % (k, v[0], v[1]) for k, v in ok.items())]
    open(os.path.join(ROOT, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for mode in ('direct', 'assisted'):
        for h in sorted({r['heading'] for r in rows}):
            rs = sorted([r for r in rows if r['mode'] == mode and r['heading'] == h],
                        key=lambda r: r['friction'])
            if not rs:
                continue
            fr = [r['friction'] for r in rs]
            kw = dict(color=COLOR[mode], ls=STYLE.get(h, '-'), marker='o',
                      label='%s, %g deg' % (mode, h))
            ax[0].plot(fr, [r['peak_F_perp_N'] for r in rs], **kw)
            for r in rs:
                if r['status'] != 'success':
                    ax[0].plot(r['friction'], r['peak_F_perp_N'], 'kx', ms=10)
            ax[1].plot(fr, [r['final_opening_m'] for r in rs], **kw)
            ax[2].plot(fr, [r['mean_F_par'] for r in rs], **kw)
    ax[0].axhline(30, ls='--', color='k', lw=1)
    ax[0].set_ylabel('peak off-axis force [N] (x = handle lost)')
    ax[1].set_ylabel('opening reached [m]')
    ax[2].set_ylabel('mean pulling force while moving [N]')
    for a in ax:
        a.set_xlabel('drawer joint friction [N]')
        a.grid(alpha=0.3)
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'friction.png'), dpi=140)
    print('saved', os.path.join(ROOT, 'friction.png'))


if __name__ == '__main__':
    main()
