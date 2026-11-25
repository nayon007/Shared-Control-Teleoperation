#!/usr/bin/env python3
"""Deterministic start-up of the step 3 simulation (run by sim.launch.py).

Gazebo starts paused with the FR3 already in its start configuration. This
node
  1. loads and configures the controllers while paused,
  2. unpauses physics and activates them in one switch (the arm has no time
     to sag, so it never pushes into the mechanism),
  3. commands the start configuration and waits until the arm holds it to
     within 2 mrad (about 1.5 mm at the TCP) for 0.5 s,
  4. welds the hand to the handle (/sc_weld/attach) and re-commands the hold
     to the configuration actually welded, so the hold controller does not
     keep pulling towards a target the weld no longer allows (pre-load),
  5. with --controller impedance, switches from the joint hold to the
     Cartesian impedance controller (step 4), which starts at the current
     pose, so the arm does not move,
  6. publishes /sim_ready = true (transient local) and keeps it alive.
"""

import argparse
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CONTROLLERS = ['joint_state_broadcaster', 'gripper_controller', 'arm_hold_controller']
IMPEDANCE = 'cartesian_impedance_controller'
CM = '/controller_manager'


class Startup(Node):

    def __init__(self, names, q_start):
        super().__init__('sim_startup')
        self.names, self.q_start = names, np.asarray(q_start)
        self.q = self.dq = None
        self.create_subscription(JointState, '/joint_states', self._joint_states, 10)
        self.ready_pub = self.create_publisher(
            Bool, '/sim_ready', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready_pub.publish(Bool(data=False))

    def _joint_states(self, msg):
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if all(n in pos for n in self.names):
            self.q = np.array([pos[n] for n in self.names])
            self.dq = np.array([vel.get(n, 0.0) for n in self.names])

    def call(self, srv_type, name, request, timeout=30.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('service %s not available' % name)
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError('service %s did not answer' % name)
        return future.result()

    def hold(self, q, seconds):
        self.hold_pub.publish(JointTrajectory(joint_names=self.names, points=[JointTrajectoryPoint(
            positions=np.asarray(q).tolist(), velocities=[0.0] * len(self.names),
            time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))]))

    def step(self, text):
        self.get_logger().info(text)

    def run(self, weld, controller='hold', gravity_term=False):
        names = CONTROLLERS + ([IMPEDANCE] if controller == 'impedance' else [])
        for name in names:
            if not self.call(LoadController, CM + '/load_controller',
                             LoadController.Request(name=name)).ok:
                raise RuntimeError('could not load ' + name)
            if name == IMPEDANCE:
                value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=gravity_term)
                self.call(SetParameters, '/%s/set_parameters' % name, SetParameters.Request(
                    parameters=[Parameter(name='gravity_compensation', value=value)]))
            if not self.call(ConfigureController, CM + '/configure_controller',
                             ConfigureController.Request(name=name)).ok:
                raise RuntimeError('could not configure ' + name)
        self.step('controllers loaded and configured (physics paused)')

        self.call(Empty, '/unpause_physics', Empty.Request())
        req = SwitchController.Request(activate_controllers=CONTROLLERS,
                                       strictness=SwitchController.Request.STRICT,
                                       activate_asap=True)
        req.timeout = Duration(sec=5)
        if not self.call(SwitchController, CM + '/switch_controller', req).ok:
            raise RuntimeError('could not activate the controllers')
        self.step('physics running, controllers active')

        # Hold exactly the IK start configuration.
        self.hold_pub = self.create_publisher(
            JointTrajectory, '/arm_hold_controller/joint_trajectory', 1)
        while self.hold_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.hold(self.q_start, 1.0)

        settled_since, t0, err = None, time.time(), np.inf
        while time.time() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.q is None:
                continue
            err = np.abs(self.q - self.q_start).max()
            # Gazebo joint velocities jitter by ~0.02 rad/s at rest.
            if err < 2e-3 and np.abs(self.dq).max() < 0.05:
                settled_since = settled_since or time.time()
                if time.time() - settled_since > 0.5:
                    break
            else:
                settled_since = None
        else:
            raise RuntimeError('arm did not settle at the start pose (error %.4f rad)' % err)
        self.step('holding start pose (max joint error %.5f rad)' % err)

        if weld:
            res = self.call(Trigger, '/sc_weld/attach', Trigger.Request())
            if not res.success:
                raise RuntimeError('weld failed: ' + res.message)
            rclpy.spin_once(self, timeout_sec=0.05)
            self.hold(self.q, 0.1)
            self.step(res.message)
        if controller == 'impedance':
            time.sleep(0.5)
            req = SwitchController.Request(activate_controllers=[IMPEDANCE],
                                           deactivate_controllers=['arm_hold_controller'],
                                           strictness=SwitchController.Request.STRICT,
                                           activate_asap=True)
            req.timeout = Duration(sec=5)
            if not self.call(SwitchController, CM + '/switch_controller', req).ok:
                raise RuntimeError('could not switch to the impedance controller')
            self.step('Cartesian impedance controller active (gravity term %s)'
                      % ('on' if gravity_term else 'off'))
        self.ready_pub.publish(Bool(data=True))
        self.step('simulation ready')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--positions-file', required=True)
    ap.add_argument('--robot-type', default='fr3')
    ap.add_argument('--weld', default='true')
    ap.add_argument('--controller', default='hold', choices=['hold', 'impedance'])
    ap.add_argument('--gravity-term', default='false')
    args, _ = ap.parse_known_args()
    q0 = yaml.safe_load(open(args.positions_file))['initial_positions']
    names = ['%s_joint%d' % (args.robot_type, i) for i in range(1, 8)]

    rclpy.init()
    node = Startup(names, [q0['joint%d' % i] for i in range(1, 8)])
    try:
        node.run(args.weld.lower() == 'true', args.controller,
                 args.gravity_term.lower() == 'true')
    except RuntimeError as e:
        node.get_logger().error(str(e))
        rclpy.shutdown()
        return 1
    rclpy.spin(node)        # keep /sim_ready available
    return 0


if __name__ == '__main__':
    sys.exit(main())
