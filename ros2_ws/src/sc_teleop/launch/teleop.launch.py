"""Shared-control teleoperation in Gazebo (steps 4 to 6), one command.

    ros2 launch sc_teleop teleop.launch.py                         # drawer, assisted, keyboard
    ros2 launch sc_teleop teleop.launch.py mode:=direct
    ros2 launch sc_teleop teleop.launch.py input:=dualsense        # joy_node + haptics
    ros2 launch sc_teleop teleop.launch.py kind:=door
    ros2 launch sc_teleop teleop.launch.py input:=script heading:=10 mode:=direct

Starts the simulation with the Cartesian impedance controller (sc_control),
then the shared-control node. With input:=keyboard, run
`python3 ~/SCE_571_ws/scripts/keyboard_joy.py` in a second terminal.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The drawer sits 10 cm further out than in Phase 3: at x = 0.7 the last
# 5 cm of the stroke fold the elbow (joint 4) close to its limit, where the
# arm is nearly singular and a compliant controller cannot hold the handle.
DEFAULT_X = {'drawer': '0.8', 'door': '0.7'}


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    kind, inp = arg('kind'), arg('input')
    if inp not in ('keyboard', 'dualsense', 'script'):
        raise RuntimeError('input must be keyboard, dualsense or script')
    sim = os.path.join(get_package_share_directory('sc_gazebo'), 'launch', 'sim.launch.py')
    cfg = os.path.join(get_package_share_directory('sc_teleop'), 'config', 'shared_control.yaml')
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
        'kind': kind, 'x': arg('x') or DEFAULT_X[kind], 'y': arg('y'), 'z': arg('z'),
        'yaw': arg('yaw'), 'radius': arg('radius'), 'gui': arg('gui'),
        'friction': arg('friction'), 'damping': arg('damping'),
        'ideal_gravity_comp': 'true', 'controller': 'impedance'}.items())]
    params = {'mode': arg('mode'), 'operator': 'script' if inp == 'script' else 'joy',
              'heading_deg': float(arg('heading')), 'trial': inp == 'script',
              'out': arg('out'), 'log': arg('log')}
    if arg('mode') == 'direct':
        params['perp_leak'] = 0.0
    if arg('perp_leak'):
        params['perp_leak'] = float(arg('perp_leak'))
    actions.append(Node(package='sc_teleop', executable='shared_control', output='screen',
                        parameters=[cfg, params]))
    record = arg('record') == 'true' or (arg('record') == 'auto' and inp != 'script')
    if record:
        name = '%s_%s_%s_f%s' % (time.strftime('%Y%m%d_%H%M%S'), arg('mode'), kind,
                                 arg('friction'))
        if inp == 'script':
            name += '_h' + arg('heading')
        out = os.path.join(os.path.expanduser(arg('sessions_dir')), name)
        actions.append(Node(package='sc_teleop', executable='session_recorder', output='screen',
                            parameters=[{'out_dir': out, 'mode': arg('mode'), 'kind': kind,
                                         'friction': float(arg('friction')),
                                         'damping': float(arg('damping')),
                                         'yaw': float(arg('yaw')), 'input': inp,
                                         'video': arg('video') == 'true'}]))
    if inp == 'dualsense':
        actions += [
            Node(package='joy', executable='joy_node', output='screen',
                 parameters=[{'autorepeat_rate': 50.0, 'deadzone': 0.0}],
                 # kernel driver (hid-playstation), not SDL's own HID driver: same
                 # axis order as the code expects, and the HID channel stays free for
                 # the haptics (pydualsense)
                 additional_env={'SDL_JOYSTICK_HIDAPI': '0'}),
            Node(package='sc_teleop', executable='dualsense_haptics', output='screen'),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='assisted', description='direct | assisted'),
        DeclareLaunchArgument('input', default_value='keyboard',
                              description='keyboard | dualsense | script'),
        DeclareLaunchArgument('heading', default_value='0.0',
                              description='scripted operator heading error [deg]'),
        DeclareLaunchArgument('kind', default_value='drawer', description='drawer | door'),
        DeclareLaunchArgument('x', default_value='', description='default 0.8 drawer, 0.7 door'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        DeclareLaunchArgument('radius', default_value='0.6'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('friction', default_value='3.0',
                              description='drawer / door joint friction [N | Nm]'),
        DeclareLaunchArgument('damping', default_value='5.0',
                              description='drawer / door joint damping [Ns/m | Nms/rad]'),
        DeclareLaunchArgument('record', default_value='auto',
                              description='auto (on for keyboard and dualsense) | true | false'),
        DeclareLaunchArgument('video', default_value='true', description='record the video too'),
        DeclareLaunchArgument('sessions_dir', default_value='~/SCE_571_ws/results/sessions'),
        DeclareLaunchArgument('perp_leak', default_value='',
                              description='override the off-axis leak [1/s]; 0 = paper'),
        DeclareLaunchArgument('out', default_value='', description='metrics JSON (script)'),
        DeclareLaunchArgument('log', default_value='', description='time series CSV'),
        OpaqueFunction(function=setup),
    ])
