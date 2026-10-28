#!/usr/bin/env python3
"""Step 3 validation against a running `sim.launch.py` (inside the container).

    ros2 run sc_gazebo weld_check.py [--out ~/SCE_571_ws/results/step3] [--offset 0.01]
    ros2 run sc_gazebo weld_check.py --demo     # only open and close along the path

The mechanism geometry is read from /mechanism/robot_description, the robot
from /robot_description. Three tests:

  1. Wrench measurement: a known effort pushes the mechanism open while the
     arm holds still; the robot's wrench on the handle (/sc_weld/handle_wrench)
     must oppose it with the right size: for the drawer the force along the
     axis, for the door the moment about the hinge axis (the weld transmits a
     moment as well as a force, and the published torque is about the
     mech_moving origin, which for the door is the hinge). Coulomb friction in
     the mechanism joint may carry up to `friction`, so the expectation is a
     band.
  2. Closed chain, on-axis: the arm follows the true opening path (IK) with
     the joint hold controller; the mechanism must follow and F_perp stay small.
  3. Off-axis: the same motion with a horizontal error perpendicular to the
     path that grows to --offset; this is the situation of the paper's
     Sec. 3.4, the error is converted into internal force F_perp. It is judged
     on the horizontal part of F_perp (the direction of the injected error):
     the temporary joint PID hold has no gravity compensation, so the arm
     also drifts vertically as its gravity load changes along the path, which
     shows up as vertical F_perp in both runs (removed by tau_g in step 4).

F_par and F_perp are computed with the ground-truth tangent d*(q). Results:
<out>_<kind>.csv / .png / .json and a PASS/FAIL table on stdout.
"""

import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import ApplyJointEffort
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sc_core.grasp import handle_pose, handle_tangent
from sc_core.kinematics import Chain

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def mechanism_from_urdf(xml):
    root = ET.fromstring(xml)
    joints = {j.get('name'): j for j in root.findall('joint')}
    o = joints['mech_base_joint'].find('origin')
    x, y, z = (float(v) for v in o.get('xyz').split())
    yaw = float(o.get('rpy').split()[2])
    mj = joints['mech_joint']
    kind = 'drawer' if mj.get('type') == 'prismatic' else 'door'
    radius = -float(mj.find('origin').get('xyz').split()[1]) if kind == 'door' else 0.0
    friction = float(mj.find('dynamics').get('friction'))
    return dict(kind=kind, x=x, y=y, z=z, yaw=yaw, radius=radius, friction=friction)


class Check(Node):

    def __init__(self):
        super().__init__('weld_check')
        self.robot_xml = self.mech_xml = None
        self.ready = False
        self.q = self.q_mech = self.F = self.W = None
        self.names = ['fr3_joint%d' % i for i in range(1, 8)]
        self.log, self.recording = [], False
        self.create_subscription(String, '/robot_description', self._robot, LATCHED)
        self.create_subscription(String, '/mechanism/robot_description', self._mech, LATCHED)
        self.create_subscription(Bool, '/sim_ready', self._ready, LATCHED)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(JointState, '/mechanism/joint_states', self._mech_js, 10)
        self.create_subscription(WrenchStamped, '/sc_weld/handle_wrench', self._wrench,
                                 qos_profile_sensor_data)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_hold_controller/joint_trajectory', 1)
        self.effort_cli = self.create_client(ApplyJointEffort, '/apply_joint_effort')

    def _robot(self, m): self.robot_xml = m.data
    def _mech(self, m): self.mech_xml = m.data
    def _ready(self, m): self.ready = m.data

    def _joints(self, m):
        pos = dict(zip(m.name, m.position))
        if all(n in pos for n in self.names):
            self.q = np.array([pos[n] for n in self.names])

    def _mech_js(self, m):
        self.q_mech = m.position[0]

    def _wrench(self, m):
        f, tq = m.wrench.force, m.wrench.torque
        self.F = np.array([f.x, f.y, f.z])
        self.W = np.array([f.x, f.y, f.z, tq.x, tq.y, tq.z])
        if self.recording and self.q_mech is not None:
            t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            self.log.append((t, self.q_mech, *self.F))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for(self, cond, what, timeout=120.0):
        end = time.time() + timeout
        while not cond():
            if time.time() > end:
                raise RuntimeError('timed out waiting for ' + what)
            rclpy.spin_once(self, timeout_sec=0.1)

    def mean_wrench(self, seconds):
        samples, end = [], time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            samples.append(self.W.copy())
        return np.mean(samples, axis=0)

    def effort(self, value):
        self.effort_cli.wait_for_service(timeout_sec=10.0)
        # duration -1: until removed; efforts add up, so remove by negating
        req = ApplyJointEffort.Request(joint_name='mechanism::mech_joint', effort=float(value),
                                       duration=Duration(sec=-1))
        rclpy.spin_until_future_complete(self, self.effort_cli.call_async(req), timeout_sec=5.0)

    def drive(self, qs, seconds):
        """Send a joint path and record until it has been executed + 1 s."""
        pts = [JointTrajectoryPoint(positions=q.tolist(), time_from_start=Duration(
            sec=int(t), nanosec=int((t % 1) * 1e9)))
            for q, t in zip(qs, np.linspace(seconds / len(qs), seconds, len(qs)))]
        self.traj_pub.publish(JointTrajectory(joint_names=self.names, points=pts))
        self.log, self.recording = [], True
        self.spin_for(seconds + 1.0)
        self.recording = False
        return np.array(self.log)


def split(log, mech):
    """Columns: t, q, F_par, |F_perp|, F_perp horizontal, F_z (world), with the
    ground-truth tangent d* (horizontal for both mechanisms)."""
    out = []
    for t, q, fx, fy, fz in log:
        d = handle_tangent(mech['kind'], mech['yaw'], q, mech['radius'] or 0.6)
        F = np.array([fx, fy, fz])
        f_par = F @ d
        n = np.cross([0.0, 0.0, 1.0], d)          # horizontal normal to the path
        out.append((t, q, f_par, np.linalg.norm(F - f_par * d), F @ n, fz))
    return np.array(out)


def path_configs(chain, mech, q_from, s_values, offset=0.0, tcp_offset=None):
    """IK along the mechanism path; optional horizontal error perpendicular to
    the path that grows linearly from 0 to `offset` [m]."""
    qs, q = [], q_from
    for i, s in enumerate(s_values):
        T = handle_pose(mech['kind'], mech['x'], mech['y'], mech['z'], mech['yaw'], s,
                        mech['radius'] or 0.6)
        T = T @ tcp_offset                       # rigid grasp: same hand pose relative to the handle
        if offset:
            d = handle_tangent(mech['kind'], mech['yaw'], s, mech['radius'] or 0.6)
            n = np.cross([0.0, 0.0, 1.0], d)
            T[:3, 3] += offset * (i / (len(s_values) - 1)) * n
        q, ok = chain.ik(T, q)
        if not ok:
            raise RuntimeError('IK failed at s = %.3f' % s)
        qs.append(q)
    return qs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/SCE_571_ws/results/step3'))
    ap.add_argument('--offset', type=float, default=0.01, help='off-axis error [m] in test 3')
    ap.add_argument('--stroke', type=float, default=None, help='m (drawer) or rad (door)')
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--demo', action='store_true', help='open and close on-axis, no checks')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Check()
    node.wait_for(lambda: node.ready, '/sim_ready (start sim.launch.py first)')
    node.wait_for(lambda: all(v is not None for v in (node.robot_xml, node.mech_xml, node.q,
                                                       node.q_mech, node.W)),
                  'robot description, joint states and handle wrench')
    mech = mechanism_from_urdf(node.mech_xml)
    chain = Chain.from_urdf(node.robot_xml, 'fr3_link0', 'fr3_hand_tcp')
    stroke = args.stroke or (0.20 if mech['kind'] == 'drawer' else 0.6)
    unit = 'm' if mech['kind'] == 'drawer' else 'rad'
    print('mechanism: %s' % json.dumps({k: round(v, 3) if isinstance(v, float) else v
                                         for k, v in mech.items()}))

    # Actual grasp relative to the nominal handle frame at the current opening
    # (hand possibly flipped, plus any small welding offset): every path pose
    # keeps it, so the commanded motion is exactly compatible with the weld.
    T0 = handle_pose(mech['kind'], mech['x'], mech['y'], mech['z'], mech['yaw'], node.q_mech,
                     mech['radius'] or 0.6)
    T_tcp = chain.fk(node.q)
    tcp_offset = np.linalg.inv(T0) @ T_tcp
    grasp_err = np.linalg.norm(T_tcp[:3, 3] - T0[:3, 3])
    results = {'mechanism': mech, 'grasp_position_error_m': grasp_err}

    if args.demo:
        s_demo = np.linspace(node.q_mech, node.q_mech + stroke, 30)
        node.drive(path_configs(chain, mech, node.q, s_demo, 0.0, tcp_offset), args.seconds)
        print('opened to %.3f %s' % (node.q_mech, unit))
        node.drive(path_configs(chain, mech, node.q, s_demo[::-1], 0.0, tcp_offset), args.seconds)
        rclpy.shutdown()
        return 0

    # ---- 1. wrench measurement ------------------------------------------------
    d0 = handle_tangent(mech['kind'], mech['yaw'], node.q_mech, mech['radius'] or 0.6)
    E = 10.0                    # N (drawer) or Nm (door); well above the 3 N | 3 Nm friction
    W_before = node.mean_wrench(2.0)
    node.effort(E)
    node.spin_for(3.0)          # let the hold controller's integral settle
    W_push = node.mean_wrench(2.0)
    node.effort(-E)
    node.spin_for(1.0)
    dW = W_push - W_before
    if mech['kind'] == 'drawer':
        measured, what = dW[:3] @ d0, 'F along axis'
    else:
        measured, what = dW[3:] @ np.array([0.0, 0.0, -1.0]), 'moment about hinge'
    expected = (-E, -(E - mech['friction']))       # (full, friction-reduced)
    ok1 = expected[0] * 1.15 <= measured <= expected[1] * 0.85
    unit_w = 'N' if mech['kind'] == 'drawer' else 'Nm'
    results['wrench_test'] = dict(effort=E, quantity=what, measured=measured,
                                  expected_range=expected, passed=bool(ok1))

    # ---- 2. on-axis and 3. off-axis drives ------------------------------------
    s_on = np.linspace(node.q_mech, node.q_mech + stroke, 30)
    q_start = node.q.copy()
    log_on = split(node.drive(path_configs(chain, mech, q_start, s_on, 0.0, tcp_offset),
                              args.seconds), mech)
    reached_on = log_on[-1, 1] - s_on[0]
    back = path_configs(chain, mech, node.q, s_on[::-1], 0.0, tcp_offset)
    node.drive(back, args.seconds)                                       # return
    log_off = split(node.drive(path_configs(chain, mech, node.q, s_on, args.offset, tcp_offset),
                               args.seconds), mech)
    node.drive(path_configs(chain, mech, node.q, s_on[::-1], 0.0, tcp_offset), args.seconds)

    moving = lambda log: log[(log[:, 1] - s_on[0]) > 0.1 * stroke]     # noqa: E731
    on_m, off_m = moving(log_on), moving(log_off)
    on_hor, off_hor = np.abs(on_m[:, 4]), np.abs(off_m[:, 4])
    ok2 = reached_on > 0.9 * stroke
    ok3 = np.mean(off_hor) > 3.0 * np.mean(on_hor)
    stats = lambda m, h: dict(mean_F_perp=float(np.mean(m[:, 3])), peak_F_perp=float(np.max(m[:, 3])),  # noqa: E731
                              mean_abs_F_perp_horizontal=float(np.mean(h)),
                              mean_abs_F_z=float(np.mean(np.abs(m[:, 5]))),
                              mean_abs_F_par=float(np.mean(np.abs(m[:, 2]))))
    results['on_axis'] = dict(stroke=stroke, reached=reached_on, passed=bool(ok2), **stats(on_m, on_hor))
    results['off_axis'] = dict(offset_m=args.offset, passed=bool(ok3), **stats(off_m, off_hor))

    # ---- report -------------------------------------------------------------
    base = '%s_%s' % (args.out, mech['kind'])
    os.makedirs(os.path.dirname(base), exist_ok=True)
    for name, log in (('on', log_on), ('off', log_off)):
        np.savetxt('%s_%s.csv' % (base, name), log, delimiter=',',
                   header='t_sim,q_mech,F_par,F_perp_norm,F_perp_horizontal,F_z', comments='')
    with open(base + '.json', 'w') as f:
        json.dump(results, f, indent=2, default=float)
    plot(base + '.png', mech, log_on, log_off, s_on[0], args.offset, unit)

    print('\n%-34s %-40s %s' % ('test', 'result', 'pass'))
    print('-' * 82)
    print('%-34s %-40s %s' % ('grasp (TCP on handle)', '%.1f mm' % (1e3 * grasp_err), grasp_err < 0.005))
    print('%-34s %-40s %s' % ('1 wrench: %s, %.0f %s push' % (what, E, unit_w),
                              '%.2f %s (expected %.1f .. %.1f)' % (measured, unit_w, *expected), ok1))
    print('%-34s %-40s %s' % ('2 on-axis drive', 'opened %.3f / %.3f %s' % (reached_on, stroke, unit), ok2))
    print('%-34s %-40s %s' % ('3 off-axis drive (%.0f mm)' % (1e3 * args.offset),
                              'horiz. |F_perp| %.1f N vs %.1f N on-axis'
                              % (np.mean(off_hor), np.mean(on_hor)), ok3))
    print('%-34s %-40s' % ('  (vertical drift, both runs)', 'mean |F_z| %.1f / %.1f N (no tau_g yet)'
                           % (np.mean(np.abs(on_m[:, 5])), np.mean(np.abs(off_m[:, 5])))))
    print('\nsaved %s.{png,json,_on.csv,_off.csv}' % base)
    rclpy.shutdown()
    return 0 if (ok1 and ok2 and ok3 and grasp_err < 0.005) else 1


def plot(path, mech, log_on, log_off, s0, offset, unit):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    for log, label, c in ((log_on, 'on-axis path', 'C0'),
                          (log_off, 'off-axis error up to %.0f mm' % (1e3 * offset), 'C3')):
        ax[0].plot(log[:, 1] - s0, log[:, 4], c, label=label)
        ax[1].plot(log[:, 1] - s0, log[:, 5], c, label=label)
        ax[2].plot(log[:, 1] - s0, log[:, 2], c, label=label)
    ax[0].set_ylabel('horizontal F_perp on handle [N]')
    ax[1].set_ylabel('vertical F_z on handle [N]')
    ax[2].set_ylabel('F_par on handle [N]')
    for a in ax:
        a.set_xlabel('opening [%s]' % unit)
        a.grid(alpha=0.3)
        a.legend()
    fig.suptitle('Step 3: FR3 welded to the %s, driven along its path (joint PID hold)' % mech['kind'])
    fig.tight_layout()
    fig.savefig(path, dpi=120)


if __name__ == '__main__':
    sys.exit(main())
