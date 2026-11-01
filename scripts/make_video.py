#!/usr/bin/env python3
"""Build the summary video (at most 60 s) on the host.

    python3 scripts/make_video.py

Inputs: results/phase3/video/frames_h{0,2}/ and trace_h{0,2}.csv (one recorded
success and one failure), results/phase3/pilot_outcomes.png.
Output: docs/phase3_video.mp4 (1280x720, 30 fps, H.264).
Needs numpy, opencv-python, Pillow and imageio-ffmpeg.
"""

import glob
import os
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = os.path.join(ROOT, 'results', 'phase3', 'video')
OUT = os.path.join(ROOT, 'docs', 'phase3_video.mp4')
W, H, FPS = 1280, 720, 30
SLOW = 4              # slow motion factor
CAM_FPS = 15.0
CROP = (170, 215, 1070, 721)   # 16:9 region of the 1280x800 camera image
BG = (24, 28, 34)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def centered(draw, y, text, f, fill=(255, 255, 255)):
    x = (W - draw.textlength(text, font=f)) / 2
    draw.text((x, y), text, font=f, fill=fill)


def card(lines):
    """lines: (text, size, bold, colour)"""
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    y = (H - sum(s * 1.55 for _, s, _, _ in lines)) / 2
    for text, size, bold, colour in lines:
        centered(d, y, text, font(size, bold), colour)
        y += size * 1.55
    return np.array(im)


def caption(im, text, sub=None):
    """Dark band with a caption at the bottom of an RGB PIL image."""
    d = ImageDraw.Draw(im, 'RGBA')
    d.rectangle((0, H - 112, W, H), fill=(0, 0, 0, 205))
    centered(d, H - 100, text, font(30, True))
    if sub:
        centered(d, H - 56, sub, font(24), (205, 212, 220))
    return im


def gauge(im, force, opening, limit_hit):
    """Live off-axis force bar and opening readout, top left."""
    d = ImageDraw.Draw(im, 'RGBA')
    x, y, w, h = 28, 26, 400, 28
    d.rectangle((x - 16, y - 14, x + w + 44, y + 110), fill=(0, 0, 0, 190))
    d.text((x, y - 4), 'Off-axis force at the handle', font=font(21), fill=(255, 255, 255))
    y += 30
    d.rectangle((x, y, x + w, y + h), outline=(255, 255, 255, 230), width=2)
    frac = min(force / 30.0, 1.0)
    col = (235, 70, 60) if frac > 0.75 else (240, 180, 50) if frac > 0.4 else (80, 200, 120)
    d.rectangle((x + 2, y + 2, x + 2 + (w - 4) * frac, y + h - 2), fill=col)
    d.text((x, y + h + 8), '%4.1f N  of 30 N limit' % force, font=font(21), fill=(255, 255, 255))
    d.text((x + 262, y + h + 8), 'opening %.3f m' % opening, font=font(18), fill=(205, 212, 220))
    if limit_hit:
        d.rectangle((W // 2 - 190, 34, W // 2 + 190, 96), fill=(200, 40, 35, 235))
        centered(d, 46, 'HANDLE LOST', font(32, True))
    return im


def load_run(h):
    files = sorted(glob.glob(os.path.join(SRC, 'frames_h%d' % h, '*.png')))
    frames = [np.array(Image.open(f).convert('RGB').crop(CROP).resize((W, H), Image.LANCZOS))
              for f in files]
    # first frame with visible motion = start of the commanded motion
    base = frames[0].astype(int)
    diff = [np.abs(f.astype(int) - base).mean() for f in frames]
    start = next(i for i, v in enumerate(diff) if v > 3 * np.median(diff[:10]) + 0.05)
    trace = np.loadtxt(os.path.join(SRC, 'trace_h%d.csv' % h), delimiter=',', skiprows=1)
    return frames, start, trace


def run_clip(frames, start, trace, text, sub, hold, lost):
    """Slow-motion clip with the gauge driven by the logged trial data."""
    out = []
    pre = int(0.4 * CAM_FPS)
    i0 = max(start - pre, 0)
    T = trace[-1, 0]
    n = int(((start - i0) / CAM_FPS + T) * SLOW * FPS)
    for k in range(n):
        tv = k / (SLOW * FPS)                 # time since the clip start, real seconds
        pos = i0 + tv * CAM_FPS
        a, frac = int(pos), pos - int(pos)
        b = min(a + 1, len(frames) - 1)
        img = cv2.addWeighted(frames[a], 1 - frac, frames[b], frac, 0)
        t = tv - (start - i0) / CAM_FPS
        f = float(np.interp(t, trace[:, 0], trace[:, 3])) if t > 0 else 0.0
        o = float(np.interp(t, trace[:, 0], trace[:, 1])) if t > 0 else 0.0
        out.append(np.array(caption(gauge(Image.fromarray(img), f, o, False), text, sub + ' (slowed 4×)')))
    last = caption(gauge(Image.fromarray(frames[min(start + int(T * CAM_FPS) + 1, len(frames) - 1)]),
                         float(trace[-1, 3]), float(trace[-1, 1]), lost), text, sub)
    out += [np.array(last)] * int(hold * FPS)
    return out


def side_by_side(a, b, text, sub):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    for x, fr, label, col in ((16, a, '0° error: drawer open', (120, 220, 150)),
                              (648, b, '2° error: stopped at 0.11 m', (245, 120, 110))):
        im.paste(Image.fromarray(fr).resize((616, 346), Image.LANCZOS), (x, 120))
        d.text((x + 308 - d.textlength(label, font=font(26, True)) / 2, 70), label,
               font=font(26, True), fill=col)
    return np.array(caption(im, text, sub))


def plot_slide(text, sub):
    p = Image.open(os.path.join(ROOT, 'results', 'phase3', 'pilot_outcomes.png')).convert('RGB')
    s = min(W / p.width, (H - 130) / p.height)
    p = p.resize((int(p.width * s), int(p.height * s)), Image.LANCZOS)
    im = Image.new('RGB', (W, H), (255, 255, 255))
    im.paste(p, ((W - p.width) // 2, (H - 112 - p.height) // 2))
    return np.array(caption(im, text, sub))


def main():
    f0, s0, t0 = load_run(0)
    f2, s2, t2 = load_run(2)
    print('motion starts at frames', s0, s2)
    white, grey = (255, 255, 255), (190, 198, 208)
    still = f0[0]

    seq = []
    seq += [card([('Sensitivity of Direct Teleoperation to Heading Error', 36, True, white),
                  ('when Opening a Constrained Mechanism', 36, True, white),
                  ('', 18, False, white),
                  ('Simulation study with the Franka Research 3 in Gazebo', 26, False, grey)])] * (5 * FPS)
    seq += [np.array(caption(Image.fromarray(still), 'The gripper is welded to the drawer handle',
                             'It can only move along the drawer axis. Other motion becomes force.'))] * (8 * FPS)
    seq += run_clip(f0, s0, t0, 'Heading error 0°: target reached',
                    'Scripted operator, drawer opens 0.25 m', 2.5, False)
    seq += run_clip(f2, s2, t2, 'Heading error 2°: 30 N limit reached',
                    'Same task, command 2° off the axis', 3.0, True)
    end0 = f0[min(s0 + int(t0[-1, 0] * CAM_FPS) + 1, len(f0) - 1)]
    end2 = f2[min(s2 + int(t2[-1, 0] * CAM_FPS) + 1, len(f2) - 1)]
    seq += [side_by_side(end0, end2, 'A small heading error decides the outcome',
                         'Direct teleoperation, stiff joint control')] * (6 * FPS)
    seq += [plot_slide('48 trials: 9 of 9 succeed up to 0.25°, 0 of 33 from 0.75°',
                       'At 0.5°, 2 of 6 succeed')] * (9 * FPS)
    seq += [card([('The handle is lost after 2.7 to 5.7 mm of sideways error', 32, True, white),
                  ('', 18, False, white),
                  ('Next: compliant control and guidance assistance', 26, False, grey)])] * (5 * FPS)

    seconds = len(seq) / FPS
    print('duration %.1f s' % seconds)
    if seconds > 60.0:
        raise SystemExit('video longer than 60 s')

    cmd = [imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo',
           '-pix_fmt', 'rgb24', '-s', '%dx%d' % (W, H), '-r', str(FPS), '-i', '-', '-an',
           '-c:v', 'libx264', '-crf', '20', '-preset', 'medium', '-pix_fmt', 'yuv420p',
           '-movflags', '+faststart', '-map_metadata', '-1', OUT]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for fr in seq:
        p.stdin.write(fr.tobytes())
    p.stdin.close()
    p.wait()
    print('saved', OUT, '%.1f MB' % (os.path.getsize(OUT) / 1e6))


if __name__ == '__main__':
    main()
