#!/usr/bin/env python3
"""Send the FR3 to its start configuration (config/initial_positions.yaml)
through arm_hold_controller. The arm sags while Gazebo starts the
controllers, so the launch file runs this once the hold controller is active.

    ros2 run sc_description go_to_start.py [--duration 3.0]
"""

import argparse
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--robot-type', default='fr3')
    args, _ = ap.parse_known_args()

    path = os.path.join(get_package_share_directory('sc_description'),
                        'config', 'initial_positions.yaml')
    q0 = yaml.safe_load(open(path))['initial_positions']
    names = ['%s_joint%d' % (args.robot_type, i) for i in range(1, 8)]

    rclpy.init()
    node = Node('go_to_start')
    pub = node.create_publisher(JointTrajectory, '/arm_hold_controller/joint_trajectory',
                                QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = JointTrajectory(joint_names=names)
    sec = int(args.duration)
    msg.points = [JointTrajectoryPoint(
        positions=[float(q0['joint%d' % i]) for i in range(1, 8)],
        velocities=[0.0] * 7,
        time_from_start=Duration(sec=sec, nanosec=int((args.duration - sec) * 1e9)))]
    pub.publish(msg)
    node.get_logger().info('sent start configuration (%.1f s)' % args.duration)
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
