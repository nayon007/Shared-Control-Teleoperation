#!/usr/bin/env python3
"""Comparison figures, videos and GIFs of the recorded DualSense sessions (host).

    python3 scripts/make_session_figures.py

For the latest direct and assisted session at each friction level in
results/sessions/, writes into results/sessions/figures/:
  comparison.png          bars (mean and peak off-axis force, time above 30 N,
                          rumble), and the force over each session's first opening
  steering.png            how far the operator's stick pointed off the drawer
                          axis, and what the robot was sent in each mode
  first_opening_f<F>.mp4  camera and live force plot, direct (top) vs assisted
  first_opening_f<F>.gif  the same, smaller, for the README
Needs numpy, matplotlib, opencv-python, imageio-ffmpeg and Pillow.
"""

import glob
import json
import os
import subprocess
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_sessions import strokes  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'sessions')
OUT = os.path.join(ROOT, 'figures')
COLOR = {'direct': 'C3', 'assisted': 'C0'}
BGR = {'direct': (60, 60, 220), 'assisted': (200, 120, 30)}
WINDOW = 9.0        # seconds shown from the first opening stroke
FPS = 15.0


def load():
    ss = {}
    for f in sorted(glob.glob(os.path.join(ROOT, '*', 'summary.json'))):
        s = json.load(open(f))
        if s.get('input') == 'script':
            continue
        s['dir'] = os.path.dirname(f)
        s['data'] = np.genfromtxt(os.path.join(s['dir'], 'data.csv'), delimiter=',', names=True)
        ss[(s['friction'], s['mode'])] = s            # latest one per condition wins
    return ss


def first_open(s):
    for k in strokes(s['data']):
        if k['direction'] == 'open':
            return k['start_s']
    return 0.0


def steering(d):
    """Angle [deg] between the stick command and the drawer axis (x), and the
    sideways speed of the command sent to the robot, while moving."""
    m = np.hypot(d['u_x'], d['u_y']) > 0.02
    a = np.degrees(np.arctan2(np.abs(d['u_y'][m]), np.abs(d['u_x'][m])))
    return a, np.abs(d['v_y'][m]), np.abs(d['u_y'][m])


def figures(ss):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fr = sorted({k[0] for k in ss})
    modes = ['direct', 'assisted']

    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(2, 4)
    metrics = [('mean_F_perp_while_moving_N', 'mean off-axis force\nwhile moving [N]'),
               ('peak_F_perp_N', 'peak off-axis force [N]'),
               ('time_above_limit_s', 'time above 30 N [s]'),
               ('mean_rumble_while_moving', 'mean rumble while moving')]
    w = 0.38
    for i, (key, lab) in enumerate(metrics):
        ax = fig.add_subplot(gs[0, i])
        for j, m in enumerate(modes):
            vals = [ss[(f, m)][key] if (f, m) in ss else np.nan for f in fr]
            ax.bar(np.arange(len(fr)) + (j - 0.5) * w, vals, w, color=COLOR[m], label=m)
            for x, v in zip(np.arange(len(fr)) + (j - 0.5) * w, vals):
                if not np.isnan(v):
                    ax.text(x, v, '%.1f' % v, ha='center', va='bottom', fontsize=8)
        if 'F_perp' in key and 'peak' in key:
            ax.axhline(30, ls='--', color='k', lw=1)
        ax.set_xticks(range(len(fr)))
        ax.set_xticklabels(['friction %g N' % f for f in fr])
        ax.set_title(lab, fontsize=10)
        ax.grid(alpha=0.3, axis='y')
        if i == 0:
            ax.legend(fontsize=8)
    for i, f in enumerate(fr):
        ax = fig.add_subplot(gs[1, 2 * i:2 * i + 2])
        for m in modes:
            if (f, m) not in ss:
                continue
            d = ss[(f, m)]['data']
            t0 = first_open(ss[(f, m)]) - 0.5
            sel = (d['t'] >= t0) & (d['t'] <= t0 + WINDOW)
            ax.plot(d['t'][sel] - t0, d['F_perp'][sel], color=COLOR[m], label='%s: off-axis force' % m)
            ax.plot(d['t'][sel] - t0, 100 * d['opening'][sel], color=COLOR[m], ls=':',
                    label='%s: opening [cm]' % m)
        ax.axhline(30, ls='--', color='k', lw=1)
        ax.text(0.1, 31, 'handle lost (30 N)', fontsize=8)
        ax.set_title('First opening, friction %g N' % f, fontsize=10)
        ax.set_xlabel('time from the start of the push [s]')
        ax.set_ylabel('force [N] / opening [cm]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle('DualSense sessions, one operator: direct vs assisted control', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'comparison.png'), dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    bins = np.linspace(0, 90, 19)
    for (f, m), s in sorted(ss.items()):
        a, vy, uy = steering(s['data'])
        ax[0].hist(a, bins, histtype='step', lw=1.6, color=COLOR[m],
                   ls='-' if f == fr[0] else '--', density=True,
                   label='%s, %g N (median %.0f deg)' % (m, f, np.median(a)))
    ax[0].set_xlabel('stick direction off the drawer axis [deg]')
    ax[0].set_ylabel('share of time moving')
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    labels, uys, vys = [], [], []
    for (f, m), s in sorted(ss.items()):
        a, vy, uy = steering(s['data'])
        labels.append('%s\n%g N' % (m, f))
        uys.append(uy.mean())
        vys.append(vy.mean())
    x = np.arange(len(labels))
    ax[1].bar(x - 0.2, uys, 0.4, color='0.6', label='operator command (sideways part)')
    ax[1].bar(x + 0.2, vys, 0.4, color='C2', label='sent to the robot (sideways part)')
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(labels, fontsize=8)
    ax[1].set_ylabel('mean sideways speed [m/s]')
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'steering.png'), dpi=140)
    plt.close(fig)


def plot_panel(d, t0, t, w, h, mode):
    """Live plot drawn with OpenCV: off-axis force (solid) and opening (thin)."""
    img = np.full((h, w, 3), 255, np.uint8)
    L, R, T, B = 50, 15, 30, 35
    fmax = 60.0
    X = lambda tt: int(L + (w - L - R) * (tt - t0) / WINDOW)  # noqa: E731
    Y = lambda f: int(h - B - (h - T - B) * min(f, fmax) / fmax)  # noqa: E731
    for f in (0, 10, 20, 30, 40, 50, 60):
        cv2.line(img, (L, Y(f)), (w - R, Y(f)), (225, 225, 225), 1)
        cv2.putText(img, str(f), (8, Y(f) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)
    cv2.line(img, (L, Y(30)), (w - R, Y(30)), (0, 0, 0), 1)
    cv2.putText(img, 'handle lost 30 N', (L + 5, Y(30) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (0, 0, 0), 1)
    sel = (d['t'] >= t0) & (d['t'] <= t)
    if sel.sum() > 1:
        pf = np.column_stack([[X(v) for v in d['t'][sel]], [Y(v) for v in d['F_perp'][sel]]])
        po = np.column_stack([[X(v) for v in d['t'][sel]],
                              [Y(v * 100 * 60 / 35) for v in d['opening'][sel]]])
        cv2.polylines(img, [po.astype(np.int32)], False, (80, 170, 80), 1, cv2.LINE_AA)
        cv2.polylines(img, [pf.astype(np.int32)], False, BGR[mode], 2, cv2.LINE_AA)
    cv2.putText(img, '%s: off-axis force [N]   (green: opening, 0-35 cm)' % mode, (L, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
    cv2.putText(img, 'time [s]', (w // 2 - 20, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (90, 90, 90), 1)
    return img


def videos(ss):
    import imageio_ffmpeg
    from PIL import Image
    for f in sorted({k[0] for k in ss}):
        pair = [ss.get((f, 'direct')), ss.get((f, 'assisted'))]
        if None in pair:
            continue
        caps, offs, t0s = [], [], []
        for s in pair:
            cap = cv2.VideoCapture(os.path.join(s['dir'], 'video.mp4'))
            n = cap.get(cv2.CAP_PROP_FRAME_COUNT) / (cap.get(cv2.CAP_PROP_FPS) or FPS)
            caps.append(cap)
            offs.append(n - s['data']['t'][-1])     # video starts before the data clock
            t0s.append(first_open(s) - 0.5)
        frames = []
        for k in range(int(WINDOW * FPS)):
            rows = []
            for s, cap, off, t0 in zip(pair, caps, offs, t0s):
                t = t0 + k / FPS
                cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, int(round((t + off) * FPS))))
                ok, cam = cap.read()
                cam = cv2.resize(cam, (640, 400)) if ok else np.zeros((400, 640, 3), np.uint8)
                rows.append(np.hstack([cam, plot_panel(s['data'], t0, t, 560, 400, s['mode'])]))
            frames.append(np.vstack(rows))
        H, W = frames[0].shape[:2]
        mp4 = os.path.join(OUT, 'first_opening_f%g.mp4' % f)
        p = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-loglevel', 'error', '-y',
                              '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '%dx%d' % (W, H),
                              '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                              '-crf', '24', mp4], stdin=subprocess.PIPE)
        for fr in frames + [frames[-1]] * int(2 * FPS):
            p.stdin.write(fr.tobytes())
        p.stdin.close()
        p.wait()
        gif = os.path.join(OUT, 'first_opening_f%g.gif' % f)
        small = [Image.fromarray(cv2.cvtColor(cv2.resize(fr, (W // 2, H // 2)), cv2.COLOR_BGR2RGB))
                 .quantize(colors=96) for fr in frames[::2]]
        small[0].save(gif, save_all=True, append_images=small[1:] + [small[-1]] * 8,
                      duration=int(2000 / FPS), loop=0, optimize=True)
        print('saved', mp4, 'and', gif)


def main():
    os.makedirs(OUT, exist_ok=True)
    ss = load()
    if not ss:
        raise SystemExit('no DualSense or keyboard sessions in ' + ROOT)
    figures(ss)
    videos(ss)
    print('figures in', OUT)


if __name__ == '__main__':
    main()
