#!/usr/bin/env python3
"""Phase 3 baseline: direct teleoperation, no assistance (paper: v_c = u).

    ros2 run sc_gazebo baseline_direct.py --heading 5 --out trial.json   # scripted operator
    ros2 run sc_gazebo baseline_direct.py --joy                          # DualSense via joy_node

The operator's Cartesian velocity u is integrated into a TCP set-point
x_d <- x_d + dt u, solved with IK and sent to the joint position controller
(arm_hold_controller) at 50 Hz. Orientation stays fixed, so this baseline is
for the drawer; the door needs the wrist compliance of the step 4 controller.

Scripted operator: full throttle at v_max along the true opening direction
rotated horizontally by --heading degrees, the same model as the offline
simulator. Joystick: left stick x/y in the base frame, right stick y for z,
R2 is throttle and deadman.

A trial ends on success (opening >= target with no handle loss), on handle
loss (true |F_perp|, low-passed over 50 ms, above --slip-force) or on timeout.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sc_core.grasp import handle_tangent, rot_z
from sc_core.kinematics import Chain
from sc_core.metrics import compute_metrics
from sc_core.params import Params
from sc_core.teleop_mapping import user_velocity

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RATE = 50.0
LINK_SETTLE = 2.0                     # [s] of simulation time
F_FILTER = 0.002 / (0.05 + 0.002)     # first-order low-pass, 50 ms at 500 Hz
# joy_node axes for a DualSense under Linux (triggers rest at +1, pressed -1)
AX_LX, AX_LY, AX_RY, AX_R2 = 0, 1, 4, 5


class Direct(Node):

    def __init__(self):
        super().__init__('baseline_direct')
        self.robot_xml = self.mech_xml = self.q = self.q_mech = self.joy = None
        self.ready = False
        self.F = None            # low-passed force on the handle
        self.t = None            # simulation time, from the 500 Hz wrench stamps
        self.names = ['fr3_joint%d' % i for i in range(1, 8)]
        self.create_subscription(String, '/robot_description', self._robot, LATCHED)
        self.create_subscription(String, '/mechanism/robot_description', self._mech, LATCHED)
        self.create_subscription(Bool, '/sim_ready', self._ready, LATCHED)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(JointState, '/mechanism/joint_states', self._mech_js, 10)
        self.create_subscription(WrenchStamped, '/sc_weld/handle_wrench', self._wrench,
                                 qos_profile_sensor_data)
        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.pub = self.create_publisher(JointTrajectory, '/arm_hold_controller/joint_trajectory', 1)

    def _robot(self, m): self.robot_xml = m.data
    def _mech(self, m): self.mech_xml = m.data
    def _ready(self, m): self.ready = m.data
    def _joy(self, m): self.joy = m

    def _joints(self, m):
        pos = dict(zip(m.name, m.position))
        if all(n in pos for n in self.names):
            self.q = np.array([pos[n] for n in self.names])

    def _mech_js(self, m):
        self.q_mech = m.position[0]

    def _wrench(self, m):
        # /clock is only ~10 Hz, the wrench stamps are 500 Hz simulation time
        self.t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        f = m.wrench.force
        F = np.array([f.x, f.y, f.z])
        self.F = F if self.F is None else self.F + F_FILTER * (F - self.F)

    def command(self, q):
        self.pub.publish(JointTrajectory(joint_names=self.names, points=[JointTrajectoryPoint(
            positions=q.tolist(), time_from_start=Duration(nanosec=int(2e9 / RATE)))]))


def joy_velocity(joy, v_max):
    a = np.array([joy.axes[AX_LX], -joy.axes[AX_LY], -joy.axes[AX_RY]])
    g = (1.0 - joy.axes[AX_R2]) / 2.0
    return user_velocity(a, g, v_max, dz=0.08)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joy', action='store_true', help='DualSense via joy_node')
    ap.add_argument('--heading', type=float, default=0.0, help='operator heading error [deg]')
    ap.add_argument('--target', type=float, default=0.25, help='opening to reach [m]')
    ap.add_argument('--timeout', type=float, default=15.0, help='trial limit, sim time [s]')
    ap.add_argument('--slip-force', type=float, default=Params.slip_force)
    ap.add_argument('--v-max', type=float, default=Params.v_max)
    ap.add_argument('--out', help='write the metrics to this JSON file')
    ap.add_argument('--log', help='write the time series to this CSV file')
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = Direct()
    while not (n.ready and all(v is not None for v in (n.robot_xml, n.mech_xml, n.q, n.q_mech, n.F))):
        rclpy.spin_once(n, timeout_sec=0.1)

    root = ET.fromstring(n.mech_xml)
    base = [j for j in root.findall('joint') if j.get('name') == 'mech_base_joint'][0]
    yaw = float(base.find('origin').get('rpy').split()[2])
    if [j for j in root.findall('joint') if j.get('name') == 'mech_joint'][0].get('type') != 'prismatic':
        raise SystemExit('the direct baseline handles the drawer only')
    d = handle_tangent('drawer', yaw)
    u_dir = rot_z(np.deg2rad(args.heading)) @ d

    chain = Chain.from_urdf(n.robot_xml, 'fr3_link0', 'fr3_hand_tcp')
    T = chain.fk(n.q)
    q_cmd = n.q.copy()

    # A new publisher takes a moment to reach the controller. Commands sent
    # before that are lost while the set-point moves on, and the arm then
    # jumps. So hold the current pose until the link has been up for a while,
    # and only then start the trial clock.
    while n.pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.05)
    t_link = n.t
    while n.t - t_link < LINK_SETTLE:
        n.command(q_cmd)
        rclpy.spin_once(n, timeout_sec=0.02)
    q0 = n.q_mech
    log = {k: [] for k in ('t', 'opening', 'F_perp_true', 'F_par')}
    t_start, last = None, None
    status = 'timeout'

    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.002)
        t = n.t
        if last is not None and t - last < 1.0 / RATE - 1e-6:
            continue
        if last is None:
            t_start = t
        dt = 0.0 if last is None else t - last
        last = t
        elapsed = t - t_start

        if args.joy:
            u = joy_velocity(n.joy, args.v_max) if n.joy is not None else np.zeros(3)
        else:
            u = args.v_max * u_dir

        T[:3, 3] += dt * u
        q_cmd, ok = chain.ik(T, q_cmd, max_iter=30)
        if not ok:
            status = 'ik_failed'
            break
        n.command(q_cmd)

        F_par = n.F @ d
        F_perp = n.F - F_par * d
        opening = n.q_mech - q0
        log['t'].append(elapsed)
        log['opening'].append(opening)
        log['F_perp_true'].append(F_perp)
        log['F_par'].append(F_par)

        if args.joy:
            continue
        if np.linalg.norm(F_perp) > args.slip_force:
            status = 'handle_lost'
            break
        if opening >= args.target:
            status = 'success'
            break
        if elapsed > args.timeout:
            break

    m = compute_metrics(log, args.slip_force, args.target)
    m.update(status=status, heading_deg=args.heading, yaw=yaw, target_m=args.target,
             v_max=args.v_max, duration_s=log['t'][-1])
    print(json.dumps(m))
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(m, f, indent=1)
    if args.log:
        rows = np.column_stack([log['t'], log['opening'], log['F_par'],
                                np.linalg.norm(log['F_perp_true'], axis=1)])
        np.savetxt(args.log, rows, delimiter=',', header='t,opening,F_par,F_perp', comments='')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
