"""RViz check of the models (no physics).

    ros2 launch sc_description view.launch.py                    # FR3 + drawer
    ros2 launch sc_description view.launch.py kind:=door yaw:=2.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_description')
    args = [
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('kind', default_value='drawer', description='drawer | door'),
        DeclareLaunchArgument('x', default_value='0.6'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        DeclareLaunchArgument('radius', default_value='0.6'),
    ]
    robot_urdf = Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'fr3_sc.urdf.xacro']),
        ' robot_type:=', LaunchConfiguration('robot_type')])
    mech_urdf = Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'mechanism.urdf.xacro']),
        ' kind:=', LaunchConfiguration('kind'),
        ' x:=', LaunchConfiguration('x'), ' y:=', LaunchConfiguration('y'),
        ' z:=', LaunchConfiguration('z'), ' yaw:=', LaunchConfiguration('yaw'),
        ' radius:=', LaunchConfiguration('radius')])

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(robot_urdf, value_type=str)}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='mechanism',
             parameters=[{'robot_description': ParameterValue(mech_urdf, value_type=str)}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             namespace='mechanism'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'view.rviz'])]),
    ])
