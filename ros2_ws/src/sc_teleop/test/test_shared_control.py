import numpy as np
import pytest

pytest.importorskip('rclpy')

from sensor_msgs.msg import Joy  # noqa: E402

from sc_teleop.shared_control import R_to_quat, joy_velocity, quat_to_R  # noqa: E402


def test_quaternion_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = quat_to_R(q)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        q2 = R_to_quat(R)
        assert np.allclose(abs(q @ q2), 1.0, atol=1e-9)


def test_joy_untouched_trigger_is_still():
    joy = Joy(axes=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert np.allclose(joy_velocity(joy, 0.1), 0.0)


def test_joy_left_stick_full_trigger_moves_minus_x():
    joy = Joy(axes=[1.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    assert np.allclose(joy_velocity(joy, 0.1), [-0.1, 0.0, 0.0])


def test_joy_released_trigger_is_deadman():
    joy = Joy(axes=[1.0, 1.0, 0.0, 0.0, 1.0, 1.0])
    assert np.allclose(joy_velocity(joy, 0.1), 0.0)
