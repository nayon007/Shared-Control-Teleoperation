"""DualSense haptic output (paper block 9).

    ros2 run sc_teleop dualsense_haptics

Reads /sc/haptics = [r, rho, ...] from shared_control and drives
  - both rumble motors with r (off-axis force and rejected command),
  - the R2 adaptive trigger with rho (force needed along the path): the
    trigger gets stiffer as the mechanism gets harder to open.

Needs the controller on USB and pydualsense (installed in the container). Runs
alongside joy_node, which reads the sticks. Without a controller it says so
and exits; keyboard_joy.py shows the same two values as bars instead.
"""

import rclpy
from std_msgs.msg import Float32MultiArray


def main():
    try:
        from pydualsense import pydualsense, TriggerModes
        ds = pydualsense()
        ds.init()
    except Exception as e:        # no device, no permission, or no hidapi
        print('no DualSense found (%s); haptics disabled' % e)
        return 1

    rclpy.init()
    node = rclpy.create_node('dualsense_haptics')
    ds.triggerR.setMode(TriggerModes.Rigid)
    state = {'last': None}

    def on_haptics(msg):
        r, rho = (max(0.0, min(1.0, v)) for v in msg.data[:2])
        level = (int(255 * r), int(255 * rho))
        if level == state['last']:
            return
        state['last'] = level
        ds.setLeftMotor(level[0])
        ds.setRightMotor(level[0])
        ds.triggerR.setForce(1, level[1])      # rigid mode: force 1 = stiffness

    node.create_subscription(Float32MultiArray, '/sc/haptics', on_haptics, 10)
    node.get_logger().info('DualSense haptics running')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ds.setLeftMotor(0)
        ds.setRightMotor(0)
        ds.triggerR.setMode(TriggerModes.Off)
        ds.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
