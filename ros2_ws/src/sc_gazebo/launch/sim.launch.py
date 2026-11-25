"""Step 3 simulation: FR3 holding the handle of a drawer or door (welded grasp).

    ros2 launch sc_gazebo sim.launch.py                         # drawer, axis towards the robot
    ros2 launch sc_gazebo sim.launch.py kind:=door radius:=0.6
    ros2 launch sc_gazebo sim.launch.py yaw:=2.8 gui:=false      # randomised axis, headless

Sequence:
  1. IK (sc_core) puts the TCP on the handle, choosing the configuration
     that can follow the opening path furthest (`stroke`).
  2. Gazebo starts PAUSED with the FR3 spawned in that configuration and the
     weld plugin loaded (world sc_weld.world).
  3. sim_startup.py loads the controllers, unpauses, activates them in the
     same instant (no sagging), holds the start pose and welds the hand to
     the handle; then /sim_ready becomes true.

Topics: /sc_weld/handle_wrench (F on the handle, world frame),
/sc_weld/attached, /sim_ready, plus everything from sc_description.
"""

import os

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sc_core.grasp import handle_pose, solve_grasp
from sc_core.kinematics import Chain

Q_READY = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
FINGER_ON_BAR = 0.013          # finger opening [m] around the 12 mm handle bar
DEFAULT_STROKE = {'drawer': 0.28, 'door': 0.75}   # trial target 0.25 m of handle travel + margin (m | rad)


def start_pose(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    kind, robot_type = arg('kind'), arg('robot_type')
    x, y, z, yaw, radius = (float(arg(k)) for k in ('x', 'y', 'z', 'yaw', 'radius'))
    stroke = float(arg('stroke')) if arg('stroke') else DEFAULT_STROKE[kind]

    urdf = xacro.process_file(
        os.path.join(get_package_share_directory('sc_description'), 'urdf', 'fr3_sc.urdf.xacro'),
        mappings={'robot_type': robot_type}).toxml()
    chain = Chain.from_urdf(urdf, '%s_link0' % robot_type, '%s_hand_tcp' % robot_type)
    path = [handle_pose(kind, x, y, z, yaw, s, radius) for s in np.linspace(0.0, stroke, 15)]
    q, flipped, reach = solve_grasp(chain, path[0], Q_READY, path=path)
    if q is None:
        raise RuntimeError('no IK solution puts the gripper on the handle at '
                           '(%.2f, %.2f, %.2f), yaw %.2f; move the mechanism' % (x, y, z, yaw))
    reach_s = np.linspace(0.0, stroke, 15)[reach - 1]

    positions = {'joint%d' % (i + 1): float(v) for i, v in enumerate(q)}
    positions['finger'] = FINGER_ON_BAR
    out = '/tmp/sc_start_pose_%d.yaml' % os.getpid()
    with open(out, 'w') as f:
        yaml.safe_dump({'initial_positions': positions}, f)

    unit = 'm' if kind == 'drawer' else 'rad'
    msg = ('start pose: q = [%s], hand flipped=%s, joint-limit margin %.2f rad, '
           'path followable to %.2f / %.2f %s'
           % (', '.join('%.3f' % v for v in q), flipped, chain.limit_margin(q), reach_s, stroke, unit))
    actions = [LogInfo(msg=msg)]
    if reach_s < stroke - 1e-6:
        actions.append(LogInfo(msg='WARNING: the arm cannot follow the full stroke from this '
                                   'mechanism pose (joint limits); see README reachability table'))

    share = get_package_share_directory('sc_gazebo')
    base = os.path.join(get_package_share_directory('sc_description'), 'launch', 'gazebo.launch.py')
    forwarded = {k: arg(k) for k in ('robot_type', 'kind', 'x', 'y', 'z', 'yaw', 'radius',
                                     'damping', 'friction', 'mass', 'gui', 'ideal_gravity_comp')}
    forwarded.update({'world': os.path.join(share, 'worlds', 'sc_weld.world'),
                      'initial_positions_file': out,
                      'paused': 'true',
                      'controllers': 'false'})
    actions += [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base),
                                 launch_arguments=forwarded.items()),
        Node(package='sc_gazebo', executable='sim_startup.py', output='screen',
             arguments=['--positions-file', out, '--robot-type', robot_type,
                        '--weld', arg('weld'), '--controller', arg('controller'),
                        '--gravity-term',
                        'false' if arg('ideal_gravity_comp') == 'true' else 'true'],
             parameters=[{'use_sim_time': True}]),
    ]
    return actions


def generate_launch_description():
    plugin_dir = os.path.join(get_package_prefix('sc_gazebo'), 'lib')
    plugin_path = os.pathsep.join(filter(None, [plugin_dir, os.environ.get('GAZEBO_PLUGIN_PATH', '')]))
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('kind', default_value='drawer', description='drawer | door'),
        DeclareLaunchArgument('x', default_value='0.7', description='handle position at q = 0 [m]'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='3.14159',
                              description='opening direction (pi = towards the robot)'),
        DeclareLaunchArgument('radius', default_value='0.6', description='door radius [m]'),
        DeclareLaunchArgument('damping', default_value='5.0'),
        DeclareLaunchArgument('friction', default_value='3.0'),
        DeclareLaunchArgument('mass', default_value='2.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('ideal_gravity_comp', default_value='false',
                              description='no gravity on the robot links (tau_g of Eq. 7)'),
        DeclareLaunchArgument('weld', default_value='true',
                              description='weld the hand to the handle after start-up'),
        DeclareLaunchArgument('controller', default_value='hold',
                              description='arm controller after the weld: hold (joint PID, '
                                          'Phase 3 baseline) | impedance (step 4)'),
        DeclareLaunchArgument('stroke', default_value='',
                              description='opening the start pose must be able to follow '
                                          '[m | rad]; default 0.28 m drawer, 0.75 rad door'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        OpaqueFunction(function=start_pose),
    ])
