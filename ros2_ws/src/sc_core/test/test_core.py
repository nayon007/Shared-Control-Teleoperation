"""Unit tests for sc_core. Run with `python3 -m unittest discover -s test`
from the package directory, or `colcon test --packages-select sc_core`."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sc_core import (ConstraintEstimator, assist, deadzone, force_split, k_perp,  # noqa: E402
                     rumble, trigger_resistance, user_velocity)
from sc_core.assistance import DIRECT, integrate_setpoint  # noqa: E402
from sc_core.mechanisms import Prismatic, Revolute  # noqa: E402
from sc_core.offline_sim import run_trial  # noqa: E402
from sc_core.params import Params  # noqa: E402


class TestEstimator(unittest.TestCase):

    def test_converges_on_noisy_line(self):
        rng = np.random.default_rng(0)
        d_true = np.array([1.0, 2.0, 0.5]) / np.linalg.norm([1.0, 2.0, 0.5])
        est = ConstraintEstimator(beta=0.1, v_min=0.01, window=30)
        for _ in range(200):
            d, c = est.update(0.05 * d_true + rng.normal(0, 0.005, 3))
        self.assertGreater(abs(d @ d_true), 0.999)
        self.assertGreater(c, 0.95)

    def test_sign_alignment_keeps_subspace(self):
        est = ConstraintEstimator(beta=1.0, v_min=0.01, window=10)
        for k in range(40):
            v = np.array([0.05, 0.0, 0.0]) * (1 if k < 20 else -1)
            d, c = est.update(v)
        self.assertAlmostEqual(abs(d[0]), 1.0, places=6)
        self.assertAlmostEqual(c, 1.0, places=6)

    def test_holds_below_speed_gate(self):
        est = ConstraintEstimator(beta=1.0, v_min=0.01, window=5)
        est.update([0.05, 0.0, 0.0])
        d, c = est.update([0.0, 0.001, 0.0])
        np.testing.assert_allclose(d, [1.0, 0.0, 0.0])

    def test_undefined_at_rest(self):
        d, c = ConstraintEstimator().update(np.zeros(3))
        self.assertIsNone(d)
        self.assertEqual(c, 0.0)

    def test_low_confidence_for_disagreeing_samples(self):
        rng = np.random.default_rng(1)
        est = ConstraintEstimator(beta=1.0, v_min=0.01, window=50)
        for _ in range(100):
            d, c = est.update(rng.normal(0, 0.1, 3))
        self.assertLess(c, 0.6)


class TestAssistance(unittest.TestCase):

    def test_k_perp_limits(self):
        self.assertEqual(k_perp(0.0), 1.0)
        self.assertAlmostEqual(k_perp(1.0, k_min=0.05), 0.05)
        self.assertAlmostEqual(k_perp(0.75, k_min=0.0, c0=0.6, c1=0.9), 0.5)

    def test_projection(self):
        u = np.array([0.1, 0.1, 0.0])
        v, u_perp = assist(u, [1.0, 0.0, 0.0], c=1.0, k_min=0.0)
        np.testing.assert_allclose(v, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(u_perp, [0.0, 0.1, 0.0])

    def test_direct_and_no_estimate_pass_through(self):
        u = np.array([0.1, 0.2, 0.3])
        np.testing.assert_allclose(assist(u, [1, 0, 0], 1.0, DIRECT)[0], u)
        np.testing.assert_allclose(assist(u, None, 0.0)[0], u)

    def test_setpoint_leak_removes_only_off_axis_error(self):
        x = np.zeros(3)
        x_d = np.array([0.02, 0.03, 0.0])
        for _ in range(5000):
            x_d = integrate_setpoint(x_d, np.zeros(3), x, [1, 0, 0], 1.0, 1e-3, perp_leak=5.0)
        self.assertAlmostEqual(x_d[0], 0.02)
        self.assertLess(abs(x_d[1]), 1e-6)


class TestHapticsAndMapping(unittest.TestCase):

    def test_force_split_is_orthogonal(self):
        F = np.array([3.0, -4.0, 1.0])
        d = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
        F_par, F_perp = force_split(F, d)
        self.assertAlmostEqual(F_perp @ d, 0.0)
        np.testing.assert_allclose(F_par * d + F_perp, F)

    def test_haptic_ranges(self):
        self.assertEqual(rumble(np.zeros(3), np.zeros(3), 0.1), 0.0)
        self.assertEqual(rumble([100.0, 0, 0], np.zeros(3), 0.1), 1.0)
        self.assertEqual(trigger_resistance(1.0), 0.0)
        self.assertEqual(trigger_resistance(-50.0), 1.0)

    def test_deadzone_and_velocity(self):
        np.testing.assert_allclose(deadzone([0.05, 0.0, 0.0]), 0.0)
        self.assertAlmostEqual(np.linalg.norm(deadzone([1.0, 0.0, 0.0])), 1.0)
        u = user_velocity([1.0, 0.0, 0.0], g=0.5, v_max=0.1)
        np.testing.assert_allclose(u, [0.05, 0.0, 0.0])


class TestOfflineSim(unittest.TestCase):
    """End-to-end check of the paper's central claim on both mechanisms."""

    def _compare(self, mech, prm):
        _, m_dir = run_trial(mech, 'direct', prm, t_max=6.0)
        _, m_ass = run_trial(mech, 'assisted', prm, t_max=6.0)
        return m_dir, m_ass

    def test_drawer_assisted_lower_off_axis_force(self):
        m_dir, m_ass = self._compare(Prismatic([0.5, 0, 0.4], [0.6, 0.8, 0]), Params(perp_leak=2.0))
        self.assertTrue(m_dir['handle_lost'])
        self.assertFalse(m_ass['handle_lost'])
        self.assertLess(m_ass['mean_F_perp_N'], 0.2 * m_dir['mean_F_perp_N'])

    def test_door_assisted_lower_off_axis_force(self):
        m_dir, m_ass = self._compare(Revolute([0.5, -0.6, 0.4], 0.6, np.pi / 2), Params(perp_leak=2.0))
        self.assertLess(m_ass['mean_F_perp_N'], 0.2 * m_dir['mean_F_perp_N'])
        self.assertGreaterEqual(m_ass['final_opening_m'], 0.25)


if __name__ == '__main__':
    unittest.main()
