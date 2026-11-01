#!/usr/bin/env python3
"""Publish fixed DualSense-style /joy messages, to test baseline_direct.py --joy
without a controller (run inside the container).

    python3 ~/SCE_571_ws/scripts/fake_joy.py --lx 1 --r2 -1 --seconds 30

Axes follow joy_node: sticks are +1 for left and up; R2 rests at +1, reads -1
fully pressed and 0 until first touched.
"""

import argparse
import time

import rclpy
from sensor_msgs.msg import Joy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lx', type=float, default=0.0)
    ap.add_argument('--ly', type=float, default=0.0)
    ap.add_argument('--ry', type=float, default=0.0)
    ap.add_argument('--r2', type=float, default=1.0)
    ap.add_argument('--seconds', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('fake_joy')
    pub = node.create_publisher(Joy, '/joy', 10)
    axes = [args.lx, args.ly, 0.0, 0.0, args.ry, args.r2]
    end = time.time() + args.seconds
    while time.time() < end:
        pub.publish(Joy(axes=axes, buttons=[]))
        time.sleep(0.02)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
