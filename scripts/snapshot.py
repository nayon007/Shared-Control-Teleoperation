#!/usr/bin/env python3
"""Save one frame of the Gazebo scene camera as PNG (run inside the container).

    python3 ~/SCE_571_ws/scripts/snapshot.py ~/SCE_571_ws/results/images/x.png

If physics is paused (gazebo.launch.py paused:=true) the camera does not
render, so the world is unpaused just long enough for one frame (~0.1 s of
sim time) and paused again.
"""

import argparse
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_srvs.srv import Empty  # noqa: E402


class Snapshot(Node):

    def __init__(self, topic):
        super().__init__('scene_snapshot')
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 1)

    def _cb(self, msg):
        if self.frame is None:
            ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 4
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            img = img[:, :msg.width * ch].reshape(msg.height, msg.width, ch)[..., :3]
            self.frame = img[..., ::-1] if msg.encoding.startswith('bgr') else img

    def physics(self, service):
        cli = self.create_client(Empty, service)
        if cli.wait_for_service(timeout_sec=5.0):
            rclpy.spin_until_future_complete(self, cli.call_async(Empty.Request()), timeout_sec=5.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--topic', default='/scene_camera/image_raw')
    ap.add_argument('--keep-running', action='store_true',
                    help='do not pause physics again afterwards')
    args = ap.parse_args()

    rclpy.init()
    node = Snapshot(args.topic)
    node.physics('/unpause_physics')
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.frame is not None:
            break
    if not args.keep_running:
        node.physics('/pause_physics')
    if node.frame is None:
        print('no image on', args.topic, file=sys.stderr)
        return 1
    mpimg.imsave(args.out, node.frame)
    print('saved', args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
