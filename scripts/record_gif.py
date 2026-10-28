#!/usr/bin/env python3
"""Record the Gazebo scene camera to an animated GIF (run inside the container),
optionally driving the mechanism joint with a test effort meanwhile.

    python3 ~/SCE_571_ws/scripts/record_gif.py out.gif --seconds 6 --effort 4.0

--effort is a force [N] for the drawer or a torque [Nm] for the door, applied
to mechanism::mech_joint through gazebo_ros_force_system for the whole clip
(it must exceed the joint's Coulomb friction, 3 N / 3 Nm by default).
"""

import argparse
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import ApplyJointEffort
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):

    def __init__(self, topic, every):
        super().__init__('gif_recorder')
        self.frames, self.every, self.count, self.recording = [], every, 0, False
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        if not self.recording:
            return
        self.count += 1
        if (self.count - 1) % self.every:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        self.frames.append(PILImage.fromarray(img.copy()))

    def _call(self, srv_type, name, req):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('%s not available' % name)
            return
        rclpy.spin_until_future_complete(self, cli.call_async(req), timeout_sec=5.0)

    def apply_effort(self, joint, effort):
        # Negative duration = until removed. This avoids start_time, which is
        # in simulation time and silently does nothing if given in wall time.
        # Efforts on a joint add up, and /clear_joint_efforts did not remove
        # them in our tests, so an effort is removed by applying its negative.
        self._call(ApplyJointEffort, '/apply_joint_effort', ApplyJointEffort.Request(
            joint_name=joint, effort=effort, duration=Duration(sec=-1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--topic', default='/scene_camera/image_raw')
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--effort', type=float, default=0.0)
    ap.add_argument('--joint', default='mechanism::mech_joint')
    ap.add_argument('--every', type=int, default=2, help='keep every n-th camera frame')
    ap.add_argument('--width', type=int, default=640)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic, args.every)
    node.recording = True
    if args.effort:
        node.apply_effort(args.joint, args.effort)
    t_end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.recording = False
    if args.effort:
        node.apply_effort(args.joint, -args.effort)
    if not node.frames:
        print('no frames received on', args.topic, file=sys.stderr)
        return 1

    w, h = node.frames[0].size
    size = (args.width, round(h * args.width / w))
    frames = [f.resize(size, PILImage.LANCZOS) for f in node.frames]
    fps = len(frames) / args.seconds
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    print('saved %s (%d frames, %.1f fps)' % (args.out, len(frames), fps))
    return 0


if __name__ == '__main__':
    sys.exit(main())
