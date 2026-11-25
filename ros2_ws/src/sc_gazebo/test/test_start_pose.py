"""Start pose on the real FR3 model: forward kinematics against the known
"ready" pose, and the IK grasp for the default drawer / door scenes,
including drawers with a tilted axis (randomised trials)."""

import os
import subprocess

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from sc_core.grasp import FLIP_Z, handle_pose, solve_grasp
from sc_core.kinematics import Chain

Q_READY = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]


@pytest.fixture(scope='module')
def chain():
    xacro_file = os.path.join(get_package_share_directory('sc_description'), 'urdf',
                              'fr3_sc.urdf.xacro')
    urdf = subprocess.run(['xacro', xacro_file], check=True, capture_output=True, text=True).stdout
    return Chain.from_urdf(urdf, 'fr3_link0', 'fr3_hand_tcp')


def test_fk_ready_pose(chain):
    # Franka "ready" pose: TCP 0.307 m in front of the base, 0.487 m high, pointing down.
    T = chain.fk(Q_READY)
    np.testing.assert_allclose(T[:3, 3], [0.3069, 0.0, 0.4869], atol=5e-4)
    np.testing.assert_allclose(T[:3, 2], [0.0, 0.0, -1.0], atol=1e-3)


@pytest.mark.parametrize('kind,x,yaw,stroke', [
    ('drawer', 0.7, np.pi, 0.28),
    ('drawer', 0.7, np.pi - 0.4, 0.28),
    ('drawer', 0.7, np.pi + 0.4, 0.28),
    ('door', 0.7, np.pi, 0.75),
    # step 4 onwards: drawer 10 cm further out (teleop.launch.py default)
    ('drawer', 0.8, np.pi, 0.28),
    ('drawer', 0.8, np.pi - 0.4, 0.28),
    ('drawer', 0.8, np.pi + 0.4, 0.28),
])
def test_grasp_and_path(chain, kind, x, yaw, stroke):
    s = np.linspace(0.0, stroke, 15)
    path = [handle_pose(kind, x, 0.0, 0.4, yaw, v, 0.6) for v in s]
    q, flipped, reach = solve_grasp(chain, path[0], Q_READY, path=path)
    assert q is not None
    T_goal = path[0] @ FLIP_Z if flipped else path[0]
    T = chain.fk(q)
    assert np.linalg.norm(T[:3, 3] - T_goal[:3, 3]) < 1e-3
    assert reach == len(path), 'start pose cannot follow the whole stroke'
    assert chain.limit_margin(q) > 0.1
