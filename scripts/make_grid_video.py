#!/usr/bin/env python3
"""Side-by-side video of recorded sessions, played in sync (host).

    python3 scripts/make_grid_video.py out.mp4 DIR1 DIR2 [DIR3 DIR4]

Each DIR holds a video.mp4 from session_recorder. Two directories give a
1x2 row, four give a 2x2 grid. A finished video holds its last frame. Output
is H.264 (plays in browsers and on GitHub); needs opencv-python and
imageio-ffmpeg.
"""

import subprocess
import sys

import cv2
import imageio_ffmpeg
import numpy as np


def main():
    out, dirs = sys.argv[1], sys.argv[2:]
    if len(dirs) not in (2, 4):
        raise SystemExit('give 2 or 4 session directories')
    caps = [cv2.VideoCapture(d.rstrip('/') + '/video.mp4') for d in dirs]
    fps = caps[0].get(cv2.CAP_PROP_FPS) or 15.0
    w, h = 640, 400
    last = [np.zeros((h, w, 3), np.uint8) for _ in caps]
    frames = []
    while True:
        alive = False
        for i, c in enumerate(caps):
            ok, f = c.read()
            if ok:
                last[i], alive = cv2.resize(f, (w, h)), True
        if not alive:
            break
        rows = [np.hstack(last[:2])] + ([np.hstack(last[2:])] if len(last) == 4 else [])
        frames.append(np.vstack(rows))
    for _ in range(int(2 * fps)):          # hold the end for 2 s
        frames.append(frames[-1])
    H, W = frames[0].shape[:2]
    p = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-loglevel', 'error', '-y',
                          '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '%dx%d' % (W, H),
                          '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                          '-crf', '24', out], stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(f.tobytes())
    p.stdin.close()
    p.wait()
    print('saved %s (%d frames, %.1f s)' % (out, len(frames), len(frames) / fps))


if __name__ == '__main__':
    main()
