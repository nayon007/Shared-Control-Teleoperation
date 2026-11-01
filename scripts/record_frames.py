#!/usr/bin/env python3
"""Save the Gazebo scene camera as numbered PNG frames (run inside the container).

    python3 ~/SCE_571_ws/scripts/record_frames.py out_dir --seconds 7
"""

import argparse
import os

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--seconds', type=float, default=7.0)
    ap.add_argument('--topic', default='/scene_camera/image_raw')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Node('frame_recorder')
    frames = []
    node.create_subscription(Image, args.topic, frames.append, 30)
    end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    for i, m in enumerate(frames):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        PILImage.fromarray(a.reshape(m.height, m.width, 3)).save(
            os.path.join(args.out, 'f%04d.png' % i), compress_level=1)
    print('saved', len(frames), 'frames')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
