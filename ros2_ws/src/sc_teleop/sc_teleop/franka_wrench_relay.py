"""Republish Franka's external-wrench estimate as geometry_msgs/WrenchStamped
(step 8, UNTESTED on hardware).

    ros2 run sc_teleop franka_wrench_relay

franka_ros2 publishes franka_msgs/FrankaRobotState on
/franka_robot_state_broadcaster/robot_state; its o_f_ext_hat_k field is the
estimated external wrench in the base frame (O_F_ext_hat_K of libfranka).
This node republishes it on /sc/external_wrench for shared_control.
"""

import rclpy
from geometry_msgs.msg import WrenchStamped


def main():
    try:
        from franka_msgs.msg import FrankaRobotState
    except ImportError:
        print('franka_msgs not found: build franka_ros2 in this workspace first')
        return 1
    rclpy.init()
    node = rclpy.create_node('franka_wrench_relay')
    pub = node.create_publisher(WrenchStamped, '/sc/external_wrench', 10)

    def on_state(msg):
        w = msg.o_f_ext_hat_k
        pub.publish(w if isinstance(w, WrenchStamped) else WrenchStamped(wrench=w))

    node.create_subscription(FrankaRobotState, '/franka_robot_state_broadcaster/robot_state',
                             on_state, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    main()
