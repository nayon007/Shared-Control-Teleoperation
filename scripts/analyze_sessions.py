#!/usr/bin/env python3
"""Summarise recorded driving sessions (DualSense or keyboard) on the host.

    python3 scripts/analyze_sessions.py [--h264]

Reads results/sessions/*/summary.json and data.csv (written by
session_recorder) and writes into results/sessions/:
  sessions.md       one row per session, then direct vs assisted per friction
  sessions.png      off-axis force and opening over time for every session
  <session>/plot.png
With --h264, also converts each video.mp4 to video_h264.mp4, which plays in a
browser and on GitHub (needs imageio-ffmpeg).
"""

import argparse
import glob
import json
import os
import subprocess
from collections import defaultdict

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'sessions')


def load():
    out = []
    for f in sorted(glob.glob(os.path.join(ROOT, '*', 'summary.json'))):
        s = json.load(open(f))
        s['dir'] = os.path.dirname(f)
        s['name'] = os.path.basename(s['dir'])
        csv = os.path.join(s['dir'], 'data.csv')
        s['data'] = np.genfromtxt(csv, delimiter=',', names=True) if os.path.exists(csv) else None
        if s['data'] is not None and s['data'].size > 1:
            out.append(s)
    return out


def strokes(d, gap=0.5, v_on=0.005):
    """Split a session into strokes: stretches where the operator commands
    motion, joined across pauses shorter than `gap` seconds."""
    t = d['t']
    speed = np.sqrt(d['u_x'] ** 2 + d['u_y'] ** 2 + d['u_z'] ** 2)
    on = speed > v_on
    out, start, last_on = [], None, None
    for i in range(len(t)):
        if on[i]:
            if start is None:
                start = i
            last_on = i
        elif start is not None and t[i] - t[last_on] > gap:
            out.append((start, last_on))
            start = None
    if start is not None:
        out.append((start, last_on))
    res = []
    for a, b in out:
        if t[b] - t[a] < 0.3:
            continue
        seg = slice(a, b + 1)
        o0, o1 = d['opening'][a], d['opening'][b]
        res.append(dict(
            start_s=float(t[a]), duration_s=float(t[b] - t[a]),
            direction='open' if o1 > o0 + 0.01 else 'close' if o1 < o0 - 0.01 else 'none',
            from_m=float(o0), to_m=float(o1),
            peak_F_perp_N=float(d['F_perp'][seg].max()),
            mean_F_perp_N=float(d['F_perp'][seg].mean()),
            peak_F_par_N=float(np.abs(d['F_par'][seg]).max()),
            end_stop=bool(d['opening'][seg].max() > 0.345 or
                          (d['opening'][seg].min() < 0.001 and np.abs(d['F_par'][seg]).max() > 15)),
            mean_rumble=float(d['rumble'][seg].mean()),
            mean_speed=float(speed[seg].mean())))
    return res


def stroke_table(ss):
    lines = ['', '**Strokes** (stretches of commanded motion; a pause over 0.5 s starts a new one)', '',
             '| session | # | start [s] | direction | from → to [m] | duration [s] | mean speed [m/s] | '
             'peak off-axis [N] | mean off-axis [N] | peak pulling [N] | end stop hit | mean rumble |',
             '|' + '---|' * 12]
    for s in ss:
        for i, k in enumerate(strokes(s['data']), 1):
            lines.append('| %s | %d | %.1f | %s | %.3f → %.3f | %.1f | %.3f | %.1f | %.1f | %.1f | %s | %.2f |'
                         % (s['name'], i, k['start_s'], k['direction'], k['from_m'], k['to_m'],
                            k['duration_s'], k['mean_speed'], k['peak_F_perp_N'],
                            k['mean_F_perp_N'], k['peak_F_par_N'], 'yes' if k['end_stop'] else 'no',
                            k['mean_rumble']))
    return '\n'.join(lines) + '\n'


def fmt(v, f='%.2f'):
    return '-' if v is None else f % v


def table(ss):
    lines = ['| session | mode | mechanism | friction [N] | input | duration [s] | furthest opening [m] | '
             'time to 0.25 m [s] | peak off-axis [N] | mean off-axis while moving [N] | '
             'time above 30 N [s] | limit crossings |', '|' + '---|' * 12]
    for s in ss:
        lines.append('| %s | %s | %s | %g | %s | %.1f | %.3f | %s | %.1f | %.1f | %.2f | %d |' % (
            s['name'], s['mode'], s['kind'], s['friction'], s['input'], s['duration_s'],
            s['max_opening_m'], fmt(s['time_to_target_s']), s['peak_F_perp_N'],
            s['mean_F_perp_while_moving_N'], s['time_above_limit_s'], s['limit_crossings']))
    g = defaultdict(list)
    for s in ss:
        g[(s['kind'], s['friction'], s['mode'])].append(s)
    lines += ['', '**Averages per condition**', '',
              '| mechanism | friction [N] | mode | sessions | reached 0.25 m | peak off-axis [N] | '
              'mean off-axis while moving [N] | time above 30 N [s] |', '|' + '---|' * 8]
    for k in sorted(g):
        rs = g[k]
        lines.append('| %s | %g | %s | %d | %d | %.1f | %.1f | %.2f |' % (
            k[0], k[1], k[2], len(rs), sum(r['time_to_target_s'] is not None for r in rs),
            np.mean([r['peak_F_perp_N'] for r in rs]),
            np.mean([r['mean_F_perp_while_moving_N'] for r in rs]),
            np.mean([r['time_above_limit_s'] for r in rs])))
    return '\n'.join(lines) + '\n'


def plots(ss):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    color = {'direct': 'C3', 'assisted': 'C0'}
    fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for s in ss:
        d = s['data']
        lab = '%s, f %g N (%s)' % (s['mode'], s['friction'], s['name'][:15])
        ax[0].plot(d['t'], d['F_perp'], color=color.get(s['mode'], 'k'), alpha=0.7, lw=1, label=lab)
        ax[1].plot(d['t'], d['opening'], color=color.get(s['mode'], 'k'), alpha=0.7, lw=1)
        f, a = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        a[0].plot(d['t'], d['F_perp'], 'C3')
        a[0].axhline(30, ls='--', color='k', lw=1)
        a[0].set_ylabel('off-axis force [N]')
        a[1].plot(d['t'], d['opening'], 'C2')
        a[1].set_ylabel('opening [m]')
        a[2].plot(d['t'], d['rumble'], 'C4', label='rumble')
        a[2].plot(d['t'], d['trigger'], 'C1', label='R2 resistance')
        a[2].plot(d['t'], d['c'], 'C0', label='confidence c')
        a[2].set_ylabel('0 to 1')
        a[2].set_xlabel('time [s]')
        a[2].legend(fontsize=8)
        a[0].set_title('%s: %s control, friction %g N' % (s['name'], s['mode'], s['friction']))
        for x in a:
            x.grid(alpha=0.3)
        f.tight_layout()
        f.savefig(os.path.join(s['dir'], 'plot.png'), dpi=110)
        plt.close(f)
    ax[0].axhline(30, ls='--', color='k', lw=1)
    ax[0].set_ylabel('off-axis force [N]')
    ax[1].set_ylabel('opening [m]')
    ax[1].set_xlabel('time [s]')
    ax[0].legend(fontsize=7, ncol=2)
    for x in ax:
        x.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'sessions.png'), dpi=120)
    plt.close(fig)


def h264(ss):
    import imageio_ffmpeg
    exe = imageio_ffmpeg.get_ffmpeg_exe()
    for s in ss:
        src, dst = os.path.join(s['dir'], 'video.mp4'), os.path.join(s['dir'], 'video_h264.mp4')
        if os.path.exists(src) and not os.path.exists(dst):
            subprocess.run([exe, '-loglevel', 'error', '-y', '-i', src, '-c:v', 'libx264',
                            '-pix_fmt', 'yuv420p', '-crf', '26', dst], check=False)


def main():
    global ROOT
    ap = argparse.ArgumentParser()
    ap.add_argument('--h264', action='store_true')
    ap.add_argument('--root', default=ROOT)
    args = ap.parse_args()
    ROOT = args.root
    ss = load()
    if not ss:
        raise SystemExit('no recorded sessions in ' + ROOT)
    md = table(ss) + stroke_table(ss)
    open(os.path.join(ROOT, 'sessions.md'), 'w').write(md)
    plots(ss)
    if args.h264:
        h264(ss)
    print(md)


if __name__ == '__main__':
    main()
