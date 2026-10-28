#!/usr/bin/env python3
"""Convert a 24/32-bit TrueColor X window dump (xwd) to PNG, numpy + matplotlib only.

    xwd -id <window-id> -silent -out shot.xwd && python3 scripts/xwd2png.py shot.xwd shot.png
"""

import struct
import sys

import matplotlib.image as mpimg
import numpy as np


def read_xwd(path):
    data = open(path, 'rb').read()
    h = struct.unpack('>25I', data[:100])
    header_size, width, height = h[0], h[4], h[5]
    bits_per_pixel, bytes_per_line, byte_order, ncolors = h[11], h[12], h[7], h[19]
    bpp = bits_per_pixel // 8           # 3 (packed, lines padded) or 4
    if bpp not in (3, 4):
        raise ValueError('only 24/32 bits per pixel are supported')
    offset = header_size + ncolors * 12
    px = np.frombuffer(data, dtype=np.uint8, count=height * bytes_per_line, offset=offset)
    px = px.reshape(height, bytes_per_line)[:, :width * bpp].reshape(height, width, bpp)
    # byte_order 0 = LSBFirst: pixels stored as B, G, R(, X)
    return px[..., [2, 1, 0]] if byte_order == 0 else px[..., [bpp - 3, bpp - 2, bpp - 1]]


if __name__ == '__main__':
    mpimg.imsave(sys.argv[2], read_xwd(sys.argv[1]))
