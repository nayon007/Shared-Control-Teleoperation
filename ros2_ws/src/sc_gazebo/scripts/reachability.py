#!/usr/bin/env python3
"""Which mechanism poses can the FR3 open? For a grid of handle positions and
axis directions, find the start pose (same solver as sim.launch.py) and report
how far it can follow the opening path continuously.

    ros2 run sc_gazebo reachability.py [--markdown] [--kind drawer]

The robot model comes from sc_description/urdf/fr3_sc.urdf.xacro.
"""

import argparse
import os
import subprocess

import numpy as np
from ament_index_python.packages import get_package_share_directory

from sc_core.grasp import handle_pose, solve_grasp
from sc_core.kinematics import Chain

Q_READY = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
STROKE = {'drawer': 0.35, 'door': 1.2}      # full drawer travel; door up to ~70 deg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kind', choices=['drawer', 'door'], nargs='*', default=['drawer', 'door'])
    ap.add_argument('--x', type=float, nargs='*', default=[0.6, 0.7, 0.8])
    ap.add_argument('--z', type=float, nargs='*', default=[0.3, 0.4, 0.5])
    ap.add_argument('--yaw-offset', type=float, nargs='*', default=[-0.4, 0.0, 0.4],
                    help='axis direction relative to pointing at the robot [rad]')
    ap.add_argument('--radius', type=float, default=0.6)
    ap.add_argument('--markdown', action='store_true')
    args = ap.parse_args()

    xacro_file = os.path.join(get_package_share_directory('sc_description'), 'urdf',
                              'fr3_sc.urdf.xacro')
    urdf = subprocess.run(['xacro', xacro_file], check=True, capture_output=True, text=True).stdout
    chain = Chain.from_urdf(urdf, 'fr3_link0', 'fr3_hand_tcp')

    cols = ['yaw %+.1f' % dy for dy in args.yaw_offset]
    for kind in args.kind:
        unit = 'm' if kind == 'drawer' else 'rad'
        s = np.linspace(0.0, STROKE[kind], 25)
        title = '%s: followable opening [%s] of %.2f (handle at x, z; y = 0)' % (kind, unit, STROKE[kind])
        if args.markdown:
            print('\n**%s**\n\n| x [m] | z [m] | %s |\n|---|---|%s' % (title, ' | '.join(cols), '---|' * len(cols)))
        else:
            print('\n' + title + '\n%-6s %-6s ' % ('x', 'z') + ' '.join('%-9s' % c for c in cols))
        for x in args.x:
            for z in args.z:
                cells = []
                for dy in args.yaw_offset:
                    path = [handle_pose(kind, x, 0.0, z, np.pi + dy, v, args.radius) for v in s]
                    q, _, reach = solve_grasp(chain, path[0], Q_READY, path=path)
                    cells.append('—' if q is None else '%.2f' % s[reach - 1])
                if args.markdown:
                    print('| %.2f | %.2f | %s |' % (x, z, ' | '.join(cells)))
                else:
                    print('%-6.2f %-6.2f ' % (x, z) + ' '.join('%-9s' % c for c in cells), flush=True)


if __name__ == '__main__':
    main()
