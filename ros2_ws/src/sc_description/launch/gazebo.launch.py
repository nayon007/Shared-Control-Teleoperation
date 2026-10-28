"""Gazebo Classic check: world + FR3 (ros2_control) + mechanism.

    ros2 launch sc_description gazebo.launch.py
    ros2 launch sc_description gazebo.launch.py kind:=door radius:=0.5
    ros2 launch sc_description gazebo.launch.py paused:=true   # inspect start pose

Controllers: joint_state_broadcaster, gripper_controller and a temporary
joint-space PID hold (arm_hold_controller) that returns the arm to its start
pose after spawning. The Cartesian impedance controller replaces the hold in
step 4; the gripper-handle weld arrives in step 3.
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def render(path, **mappings):
    """xacro -> URDF string without XML comments.

    gazebo_ros2_control (Humble) forwards robot_description as a
    `--param robot_description:='...'` rule, and the single quotes inside
    franka_description's comments break that rule.
    """
    urdf = xacro.process_file(path, mappings={k: str(v) for k, v in mappings.items()}).toxml()
    return re.sub(r'<!--.*?-->', '', urdf, flags=re.DOTALL)


def state_publishers(context):
    share = get_package_share_directory('sc_description')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_urdf = render(os.path.join(share, 'urdf', 'fr3_sc.urdf.xacro'),
                        sim_gazebo='true', robot_type=arg('robot_type'))
    mech_urdf = render(os.path.join(share, 'urdf', 'mechanism.urdf.xacro'),
                       kind=arg('kind'), x=arg('x'), y=arg('y'), z=arg('z'),
                       yaw=arg('yaw'), radius=arg('radius'), damping=arg('damping'),
                       friction=arg('friction'), mass=arg('mass'))
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='mechanism',
             parameters=[{'robot_description': mech_urdf, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    pkg = FindPackageShare('sc_description')
    # Gazebo resolves package://franka_description/... via GAZEBO_MODEL_PATH
    share_parent = os.path.dirname(get_package_share_directory('franka_description'))
    model_path = os.pathsep.join(filter(None, [share_parent, os.environ.get('GAZEBO_MODEL_PATH', '')]))

    args = [
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('kind', default_value='drawer', description='drawer | door'),
        DeclareLaunchArgument('x', default_value='0.6'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        DeclareLaunchArgument('radius', default_value='0.6'),
        DeclareLaunchArgument('damping', default_value='5.0',
                              description='mechanism joint damping [Ns/m | Nms/rad]'),
        DeclareLaunchArgument('friction', default_value='3.0',
                              description='mechanism joint Coulomb friction [N | Nm]'),
        DeclareLaunchArgument('mass', default_value='2.0',
                              description='drawer / door leaf mass [kg]'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='start with physics paused (arm stays in its start pose)'),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': PathJoinSubstitution([pkg, 'worlds', 'sc_empty.world']),
                          'gui': LaunchConfiguration('gui'),
                          'pause': LaunchConfiguration('paused')}.items())

    # ros2_control only updates while physics steps, so controllers cannot be
    # activated in a paused world; paused:=true is for inspecting the start pose.
    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'],
                    condition=UnlessCondition(LaunchConfiguration('paused')))

    hold = spawner('arm_hold_controller')  # temporary, until step 4

    return LaunchDescription(args + [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        OpaqueFunction(function=state_publishers),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'fr3']),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', '/mechanism/robot_description', '-entity', 'mechanism']),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-file', PathJoinSubstitution([pkg, 'models', 'scene_camera', 'model.sdf']),
                        '-entity', 'scene_camera',
                        '-x', '-0.55', '-y', '1.6', '-z', '1.25',
                        '-R', '0', '-P', '0.450', '-Y', '-1.035']),
        spawner('joint_state_broadcaster'),
        spawner('gripper_controller'),
        hold,
        # The arm sags while controllers start; drive it back to the start pose.
        RegisterEventHandler(OnProcessExit(target_action=hold, on_exit=[
            Node(package='sc_description', executable='go_to_start.py',
                 parameters=[{'use_sim_time': True}])])),
    ])
