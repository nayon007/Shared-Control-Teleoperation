"""Shared-control node (paper Alg. 1) on top of the Cartesian impedance controller.

    ros2 run sc_teleop shared_control --ros-args -p mode:=assisted -p operator:=joy
    ros2 run sc_teleop shared_control --ros-args -p mode:=direct -p operator:=script -p heading_deg:=5.0

Every 10 ms of simulation time:
  1. operator command u: from /joy (DualSense or keyboard_joy.py), or a
     scripted operator pushing at v_max along the true initial direction
     turned by heading_deg,
  2. estimator: tangent d_hat and confidence c from the measured gripper
     velocity (Lutscher et al. 2010, sc_core.estimator),
  3. assistance: direct (v_c = u) or guidance fixture (sc_core.assistance),
  4. set-point x_d <- x_d + dt v_c, sent to the impedance controller,
  5. haptics: rumble r and trigger resistance rho from the measured handle
     force, published on /sc/haptics = [r, rho, c, F_par, |F_perp|],
  6. trial check and logging (ground truth only here, never in 1-5).

The controller never sees the true axis; it is read only for the metrics.

On the real FR3 (step 8) there is no weld and no mechanism ground truth:
set wrench_topic to Franka's external-wrench estimate (O_F_ext_hat_K, the
force ON the robot, hence wrench_sign -1) and ground_truth false; opening is
then the distance travelled by the gripper and off-axis force uses d_hat.
"""

import json
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float32MultiArray, String

from sc_core.assistance import ASSISTED, DIRECT, assist, integrate_setpoint
from sc_core.estimator import ConstraintEstimator
from sc_core.grasp import handle_tangent, rot_z
from sc_core.haptics import force_split, rumble, trigger_resistance
from sc_core.metrics import compute_metrics
from sc_core.params import Params
from sc_core.teleop_mapping import user_velocity

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CTRL = '/cartesian_impedance_controller'
F_FILTER = 0.002 / (0.05 + 0.002)     # 50 ms low-pass on the 500 Hz wrench
JOY_TIMEOUT = 0.5                     # [s] wall time: no /joy for longer = stop
SETTLE = 1.0                          # [s] hold before the trial clock starts
# joy_node axes for a DualSense: sticks +1 left/up; R2 +1 released, -1 pressed, 0 untouched
AX_LX, AX_LY, AX_RY, AX_R2 = 0, 1, 4, 5
STATUS_FIELDS = ['t', 'opening', 'F_perp', 'F_par', 'F_perp_est', 'c', 'rumble', 'trigger',
                 'u_x', 'u_y', 'u_z', 'v_x', 'v_y', 'v_z', 'assisted']


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def joy_velocity(joy, v_max, dz=0.08):
    a = np.array([-joy.axes[AX_LX], joy.axes[AX_LY], joy.axes[AX_RY]])
    g = 0.0 if joy.axes[AX_R2] == 0.0 else (1.0 - joy.axes[AX_R2]) / 2.0
    return user_velocity(a, g, v_max, dz=dz)


class SharedControl(Node):

    def __init__(self):
        super().__init__('shared_control')
        p = Params()
        for name, default in (
                ('mode', ASSISTED), ('operator', 'joy'), ('heading_deg', 0.0),
                ('target', 0.25), ('timeout', 15.0), ('trial', False),
                ('v_max', p.v_max), ('slip_force', p.slip_force), ('perp_leak', p.perp_leak),
                ('beta', p.beta), ('v_min', p.v_min), ('window', p.window),
                ('k_min', p.k_min), ('c0', p.c0), ('c1', p.c1),
                ('follow_yaw', True), ('out', ''), ('log', ''),
                ('wrench_topic', '/sc_weld/handle_wrench'), ('wrench_sign', 1.0),
                ('ground_truth', True), ('max_lead', 0.10)):
            self.declare_parameter(name, default)
        self.prm = {k: self.get_parameter(k).value for k in (
            'mode', 'operator', 'heading_deg', 'target', 'timeout', 'trial', 'v_max',
            'slip_force', 'perp_leak', 'beta', 'v_min', 'window', 'k_min', 'c0', 'c1',
            'follow_yaw', 'out', 'log', 'wrench_topic', 'wrench_sign', 'ground_truth', 'max_lead')}
        if self.prm['mode'] not in (DIRECT, ASSISTED):
            raise SystemExit('mode must be direct or assisted')

        self.mech_xml = self.pose = self.twist = self.F = self.t = self.s = self.joy = None
        self.ready = False
        self.joy_time = 0.0
        self.create_subscription(PoseStamped, CTRL + '/current_pose', self._pose, 10)
        self.create_subscription(TwistStamped, CTRL + '/current_twist', self._twist, 10)
        self.create_subscription(WrenchStamped, self.prm['wrench_topic'], self._wrench,
                                 qos_profile_sensor_data)
        if self.prm['ground_truth']:
            self.create_subscription(String, '/mechanism/robot_description', self._mech, LATCHED)
            self.create_subscription(Bool, '/sim_ready', self._ready, LATCHED)
            self.create_subscription(JointState, '/mechanism/joint_states', self._mech_js, 10)
        else:
            self.ready, self.s = True, 0.0
        self.create_subscription(Joy, '/joy', self._joy, 10)
        self.target_pub = self.create_publisher(PoseStamped, CTRL + '/target_pose', 10)
        self.haptics_pub = self.create_publisher(Float32MultiArray, '/sc/haptics', 10)
        # everything a recorder needs, one message per 10 ms cycle (STATUS_FIELDS)
        self.status_pub = self.create_publisher(Float32MultiArray, '/sc/status', 50)

    def _mech(self, m): self.mech_xml = m.data
    def _ready(self, m): self.ready = m.data
    def _joy(self, m):
        self.joy, self.joy_time = m, time.monotonic()
    def _mech_js(self, m): self.s = m.position[0]

    def _pose(self, m):
        o = m.pose.orientation
        self.pose = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                     quat_to_R((o.w, o.x, o.y, o.z)))

    def _twist(self, m):
        self.twist = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])

    def _wrench(self, m):
        self.t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        f = m.wrench.force
        F = self.prm['wrench_sign'] * np.array([f.x, f.y, f.z])
        self.F = F if self.F is None else self.F + F_FILTER * (F - self.F)

    def send(self, x_d, R_d):
        m = PoseStamped()
        m.header.frame_id = 'fr3_link0'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in x_d)
        q = R_to_quat(R_d)
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = \
            (float(v) for v in q)
        self.target_pub.publish(m)

    def spin_until(self, cond):
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.05)

    def mechanism(self):
        root = ET.fromstring(self.mech_xml)
        joint = {j.get('name'): j for j in root.findall('joint')}
        yaw = float(joint['mech_base_joint'].find('origin').get('rpy').split()[2])
        kind = 'drawer' if joint['mech_joint'].get('type') == 'prismatic' else 'door'
        radius = 0.6
        if kind == 'door':     # hinge at (-h, -r, 0) in the mechanism frame
            radius = abs(float(joint['mech_joint'].find('origin').get('xyz').split()[1]))
        return kind, yaw, radius

    def run(self):
        P = self.prm
        self.get_logger().info('waiting for the simulation and the impedance controller')
        gt = P['ground_truth']
        self.spin_until(lambda: self.ready and all(v is not None for v in (
            self.mech_xml if gt else 0, self.pose, self.twist, self.F, self.s)))
        self.spin_until(lambda: self.target_pub.get_subscription_count() > 0)
        if gt:
            kind, yaw, radius = self.mechanism()
            d0 = handle_tangent(kind, yaw, 0.0, radius)
        else:   # unknown mechanism: only joystick operation makes sense
            kind, yaw, radius, d0 = 'unknown', float('nan'), 1.0, np.array([1.0, 0.0, 0.0])
            P['operator'] = 'joy'
        travelled = 0.0
        u_dir = rot_z(np.deg2rad(P['heading_deg'])) @ d0
        x_d, R0 = self.pose[0].copy(), self.pose[1].copy()

        t_hold = self.t
        while rclpy.ok() and self.t - t_hold < SETTLE:
            self.send(x_d, R0)
            rclpy.spin_once(self, timeout_sec=0.02)
        s0 = self.s
        est = ConstraintEstimator(P['beta'], P['v_min'], int(P['window']))
        d_hat, c = None, 0.0
        log = {k: [] for k in ('t', 'opening', 'F_perp_true', 'F_par', 'F_perp_est', 'c',
                               'angle_err_deg', 'r', 'rho', 'u', 'v_cmd')}
        self.get_logger().info(
            'READY: %s control, %s operator, %s (yaw %.2f)%s' % (
                P['mode'], P['operator'], kind, yaw,
                ', heading error %g deg' % P['heading_deg'] if P['operator'] == 'script' else
                ', steer with the joystick or keyboard_joy.py'))

        status, last, t_start, next_report = 'timeout', None, None, 1.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.002)
            t = self.t
            if last is not None and t - last < 0.01 - 1e-6:
                continue
            dt = 0.0 if last is None else t - last
            t_start = t if t_start is None else t_start
            last, elapsed = t, t - t_start

            travelled += dt * np.linalg.norm(self.twist)
            q_m = self.s - s0
            opening = (q_m * radius if kind == 'door' else q_m) if gt else travelled
            if P['operator'] == 'script':
                u = P['v_max'] * u_dir if opening < P['target'] else np.zeros(3)
            else:
                fresh = self.joy is not None and time.monotonic() - self.joy_time < JOY_TIMEOUT
                u = joy_velocity(self.joy, P['v_max']) if fresh else np.zeros(3)

            x, R = self.pose
            d_hat, c = est.update(self.twist)
            v_cmd, u_perp = assist(u, d_hat, c, P['mode'], P['k_min'], P['c0'], P['c1'])
            x_d = integrate_setpoint(x_d, v_cmd, x, d_hat if P['mode'] == ASSISTED else None,
                                     c, dt, P['perp_leak'], P['c1'])
            # Anti-windup: at an end stop the set-point would otherwise run on
            # without limit, and a reversed command would first have to unwind
            # it before the mechanism moves. Keep it within the controller's
            # error cap, so the force is unchanged.
            e = x_d - x
            if np.linalg.norm(e) > P['max_lead']:
                x_d = x + e * P['max_lead'] / np.linalg.norm(e)
            R_d = R0
            if P['follow_yaw']:       # let the wrist turn with a door: no yaw spring
                M = R @ R0.T
                R_d = rot_z(np.arctan2(M[1, 0], M[0, 0])) @ R0
            self.send(x_d, R_d)

            F_par, F_perp = force_split(self.F, d_hat)
            r = rumble(F_perp, u_perp, P['v_max'])
            rho = trigger_resistance(F_par)
            self.haptics_pub.publish(Float32MultiArray(data=[
                float(r), float(rho), float(c), float(F_par), float(np.linalg.norm(F_perp))]))

            d_true = handle_tangent(kind, yaw, q_m, radius) if gt else d_hat
            F_par_true, F_perp_true = force_split(self.F, d_true)
            ang = np.nan if d_hat is None else np.degrees(
                np.arccos(np.clip(abs(d_hat @ d_true), 0.0, 1.0)))
            for k, v in (('t', elapsed), ('opening', opening), ('F_perp_true', F_perp_true),
                         ('F_par', F_par_true), ('F_perp_est', np.linalg.norm(F_perp)),
                         ('c', c), ('angle_err_deg', ang), ('r', r), ('rho', rho),
                         ('u', np.linalg.norm(u)), ('v_cmd', np.linalg.norm(v_cmd))):
                log[k].append(v)

            fp = np.linalg.norm(F_perp_true)
            self.status_pub.publish(Float32MultiArray(data=[float(v) for v in (
                elapsed, opening, fp, F_par_true, np.linalg.norm(F_perp), c, r, rho,
                *u, *v_cmd, 1.0 if P['mode'] == ASSISTED else 0.0)]))
            if elapsed >= next_report and (P['operator'] == 'joy' or not P['trial']):
                next_report += 1.0
                self.get_logger().info(
                    't %5.1f s  opening %.3f m  off-axis %5.1f N  along %5.1f N  c %.2f  '
                    'rumble %.2f  trigger %.2f%s' % (
                        elapsed, opening, fp, F_par_true, c, r, rho,
                        '  (no /joy messages)' if P['operator'] == 'joy' and self.joy is None
                        else '  HANDLE WOULD BE LOST (> %g N)' % P['slip_force']
                        if fp > P['slip_force'] else ''))
            if P['trial'] or P['operator'] == 'script':
                if fp > P['slip_force']:
                    status = 'handle_lost'
                    break
                if opening >= P['target']:
                    status = 'success'
                    break
                if elapsed > P['timeout']:
                    break

        # leave the arm holding where it is
        if self.pose is not None:
            self.send(self.pose[0], self.pose[1])
        m = compute_metrics(log, P['slip_force'], P['target'])
        m.update(status=status, mode=P['mode'], heading_deg=P['heading_deg'], kind=kind,
                 yaw=yaw, target_m=P['target'], duration_s=log['t'][-1] if log['t'] else 0.0,
                 mean_c=float(np.mean(log['c'])) if log['c'] else 0.0)
        print(json.dumps(m), flush=True)
        if P['out']:
            with open(P['out'], 'w') as f:
                json.dump(m, f, indent=1)
        if P['log']:
            cols = ['t', 'opening', 'F_par', 'F_perp_est', 'c', 'angle_err_deg', 'r', 'rho',
                    'u', 'v_cmd']
            rows = np.column_stack([np.asarray(log[k], dtype=float) for k in cols] +
                                   [np.linalg.norm(log['F_perp_true'], axis=1)])
            np.savetxt(P['log'], rows, delimiter=',', header=','.join(cols + ['F_perp']),
                       comments='')


def main():
    rclpy.init()
    node = SharedControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
