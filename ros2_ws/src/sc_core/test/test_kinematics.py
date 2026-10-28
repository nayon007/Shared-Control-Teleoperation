"""Tests for sc_core.kinematics and sc_core.grasp (no ROS needed)."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sc_core.grasp import FLIP_Z, grasp_rotation, handle_pose, handle_tangent, rot_z  # noqa: E402
from sc_core.kinematics import Chain, axis_angle_matrix, rotation_error  # noqa: E402

# Planar 2R arm (links 0.5 m and 0.4 m, axes along z) with a fixed tool frame.
TWO_R = """
<robot name="two_r">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tool"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="l2"/><child link="tool"/><origin xyz="0.4 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


class TestKinematics(unittest.TestCase):

    def setUp(self):
        self.chain = Chain.from_urdf(TWO_R, 'base', 'tool')

    def test_fk_matches_closed_form(self):
        q = np.array([0.3, -0.7])
        p = self.chain.fk(q)[:3, 3]
        expected = [0.5 * np.cos(0.3) + 0.4 * np.cos(-0.4),
                    0.5 * np.sin(0.3) + 0.4 * np.sin(-0.4), 0.1]
        np.testing.assert_allclose(p, expected, atol=1e-12)
        self.assertEqual(self.chain.names, ['j1', 'j2'])

    def test_jacobian_matches_finite_differences(self):
        q, eps = np.array([0.4, 0.9]), 1e-6
        J = self.chain.jacobian(q)
        for k in range(2):
            dq = np.zeros(2)
            dq[k] = eps
            dp = (self.chain.fk(q + dq)[:3, 3] - self.chain.fk(q - dq)[:3, 3]) / (2 * eps)
            np.testing.assert_allclose(J[:3, k], dp, atol=1e-8)
        np.testing.assert_allclose(J[3:], [[0, 0], [0, 0], [1, 1]])

    def test_ik_round_trip_position_and_yaw(self):
        q_true = np.array([0.8, -1.1])
        T = self.chain.fk(q_true)
        q, ok = self.chain.ik(T, np.array([0.0, -0.3]))
        self.assertTrue(ok)
        np.testing.assert_allclose(self.chain.fk(q), T, atol=1e-4)

    def test_rotation_error(self):
        R = axis_angle_matrix([0, 0, 1], 0.2)
        np.testing.assert_allclose(rotation_error(R, np.eye(3)), [0, 0, 0.2], atol=1e-12)
        R_pi = axis_angle_matrix([1, 0, 0], np.pi)
        np.testing.assert_allclose(np.abs(rotation_error(R_pi, np.eye(3))), [np.pi, 0, 0], atol=1e-6)

    def test_limit_margin(self):
        self.assertAlmostEqual(self.chain.limit_margin(np.array([2.5, 0.0])), 0.5)


class TestGrasp(unittest.TestCase):

    def test_grasp_rotation_is_proper_and_points_into_mechanism(self):
        for yaw in (np.pi, 2.6, 0.3):
            R = grasp_rotation(rot_z(yaw))
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0)
            np.testing.assert_allclose(R[:, 2], -rot_z(yaw)[:, 0], atol=1e-12)  # approach = -x_M
            np.testing.assert_allclose(R[:, 1], [0, 0, 1], atol=1e-12)          # fingers vertical

    def test_drawer_path_is_a_line_along_the_tangent(self):
        yaw = np.pi - 0.4
        p0 = handle_pose('drawer', 0.7, 0.1, 0.4, yaw, 0.0)[:3, 3]
        p1 = handle_pose('drawer', 0.7, 0.1, 0.4, yaw, 0.2)[:3, 3]
        np.testing.assert_allclose(p0, [0.7, 0.1, 0.4])
        np.testing.assert_allclose(p1 - p0, 0.2 * handle_tangent('drawer', yaw), atol=1e-12)

    def test_door_path_is_an_arc_and_tangent_matches_derivative(self):
        r, yaw, eps = 0.6, np.pi, 1e-6
        p0 = handle_pose('door', 0.7, 0.0, 0.4, yaw, 0.0, r)[:3, 3]
        np.testing.assert_allclose(p0, [0.7, 0.0, 0.4], atol=1e-12)
        for q in (0.0, 0.5):
            pa = handle_pose('door', 0.7, 0.0, 0.4, yaw, q - eps, r)[:3, 3]
            pb = handle_pose('door', 0.7, 0.0, 0.4, yaw, q + eps, r)[:3, 3]
            d = (pb - pa) / np.linalg.norm(pb - pa)
            np.testing.assert_allclose(d, handle_tangent('door', yaw, q, r), atol=1e-6)
        self.assertAlmostEqual(handle_tangent('door', yaw, 0.0, r) @ [-1, 0, 0], r / np.hypot(r, 0.05))

    def test_flip_keeps_approach_axis(self):
        T = handle_pose('drawer', 0.7, 0, 0.4, np.pi)
        np.testing.assert_allclose((T @ FLIP_Z)[:3, 2], T[:3, 2])
        np.testing.assert_allclose((T @ FLIP_Z)[:3, 1], -T[:3, 1])


if __name__ == '__main__':
    unittest.main()
