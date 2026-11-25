"""Step 8: shared control on the real FR3 (UNTESTED on hardware).

Prerequisites (see README section 10): PREEMPT_RT kernel on the robot PC,
franka_ros2 and libfranka matching the robot's system image, FCI enabled in
Desk, the user stop in reach.

    # terminal 1: franka_ros2 with its controller manager, no controller yet
    ros2 launch franka_bringup franka.launch.py robot_ip:=<FR3 IP> arm_id:=fr3 load_gripper:=true
    # terminal 2: this file (controller, operator input, haptics)
    ros2 launch sc_teleop hardware.launch.py mode:=assisted

The controller starts holding the current pose; grasp the handle first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sc_teleop'), 'config', 'fr3_hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='assisted', description='direct | assisted'),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['cartesian_impedance_controller', '--controller-manager',
                        '/controller_manager', '--param-file', cfg,
                        '--controller-type', 'sc_control/CartesianImpedanceController']),
        Node(package='sc_teleop', executable='franka_wrench_relay', output='screen'),
        Node(package='sc_teleop', executable='shared_control', output='screen',
             parameters=[cfg, {'mode': LaunchConfiguration('mode'), 'operator': 'joy'}]),
        Node(package='joy', executable='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 50.0, 'deadzone': 0.0}],
             additional_env={'SDL_JOYSTICK_HIDAPI': '0'}),  # kernel driver
        Node(package='sc_teleop', executable='dualsense_haptics', output='screen'),
    ])
