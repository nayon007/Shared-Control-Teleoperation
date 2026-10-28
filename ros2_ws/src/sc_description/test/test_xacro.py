"""The xacro files expand, and the mechanism frames follow the convention:
the handle sits at the mechanism origin (drawer) and the joint moves it
along +x_M at q = 0 (drawer and door)."""

import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

URDF_DIR = get_package_share_directory('sc_description') + '/urdf/'


def xacro(name, *args):
    out = subprocess.run(['xacro', URDF_DIR + name, *args], check=True,
                         capture_output=True, text=True).stdout
    return ET.fromstring(out)


def joints(root):
    return {j.get('name'): j for j in root.findall('joint')}


@pytest.mark.parametrize('sim', ['false', 'true'])
def test_robot_expands(sim):
    root = xacro('fr3_sc.urdf.xacro', 'sim_gazebo:=' + sim)
    j = joints(root)
    assert all('fr3_joint%d' % i in j for i in range(1, 8))
    assert ('fr3_hand_tcp' in {l.get('name') for l in root.findall('link')})
    rc = root.findall('ros2_control')
    assert len(rc) == (1 if sim == 'true' else 0)
    if rc:
        cmd = {jt.get('name'): [c.get('name') for c in jt.findall('command_interface')]
               for jt in rc[0].findall('joint')}
        assert all(cmd['fr3_joint%d' % i] == ['effort'] for i in range(1, 8))


def _vec(s):
    return np.array([float(v) for v in (s or '0 0 0').split()])


def test_drawer_axis_is_plus_x():
    j = joints(xacro('mechanism.urdf.xacro', 'kind:=drawer'))['mech_joint']
    assert j.get('type') == 'prismatic'
    np.testing.assert_allclose(_vec(j.find('axis').get('xyz')), [1, 0, 0])


@pytest.mark.parametrize('radius', [0.4, 0.8])
def test_door_handle_moves_plus_x(radius):
    root = xacro('mechanism.urdf.xacro', 'kind:=door', 'radius:=%g' % radius)
    j = joints(root)['mech_joint']
    hinge = _vec(j.find('origin').get('xyz'))
    axis = _vec(j.find('axis').get('xyz'))
    handle = [l for l in root.findall('link') if l.get('name') == 'mech_handle'][0]
    handle_in_leaf = _vec(handle.find('visual/origin').get('xyz'))
    v = np.cross(axis, handle_in_leaf)        # velocity of the handle for dq = 1
    np.testing.assert_allclose(hinge + handle_in_leaf, 0.0, atol=1e-9)  # handle at M origin
    assert v[0] / np.linalg.norm(v) > 0.99
