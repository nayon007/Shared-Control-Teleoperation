#!/usr/bin/env python3
"""Drive the robot from the keyboard instead of a DualSense (run inside the container).

    python3 ~/SCE_571_ws/scripts/keyboard_joy.py

Publishes /joy messages exactly like joy_node with a DualSense, so
`baseline_direct.py --joy` cannot tell the difference. A terminal cannot see
key releases, so keys set the stick position and it stays there until
changed; R2 is a toggle. With shared_control running, the status line also
shows the haptic feedback a DualSense would give: rumble (off-axis force)
and trigger stiffness (force needed to open).

    a / d     left stick left / right      (base -x / +x)
    w / s     left stick up / down         (base +y / -y)
    i / k     right stick up / down        (hand up / down)
    space     R2 on / off  (speed and deadman: off = robot stays still)
    1 .. 9    speed: R2 pressed 10 % .. 90 %, 0 = 100 %
    x         sticks back to centre
    q         quit
"""

import select
import sys
import termios
import time
import tty

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

STEP = 0.25     # stick change per key press


def clamp(v):
    return max(-1.0, min(1.0, v))


def status(lx, ly, ry, r2_on, level, listening, haptics=None):
    trig = 'R2 ON  %3d %%' % round(level * 100) if r2_on else 'R2 off (robot holds still)'
    bar = lambda v: '[' + ''.join('#' if abs(v) * 4 >= i else '.' for i in range(1, 5)) + ']'  # noqa: E731
    left = ('left ' if lx > 0 else 'right' if lx < 0 else '     ') + bar(lx)
    up = ('up  ' if ly > 0 else 'down' if ly < 0 else '    ') + bar(ly)
    z = ('up  ' if ry > 0 else 'down' if ry < 0 else '    ') + bar(ry)
    trial = 'trial listening' if listening else 'NO TRIAL LISTENING - start a trial or teleop.launch.py'
    feel = ''
    if haptics is not None:
        r, rho = haptics
        meter = lambda v: '[' + '#' * round(v * 5) + '.' * (5 - round(v * 5)) + ']'  # noqa: E731
        feel = ' | rumble %s trigger %s' % (meter(r), meter(rho))
    return '\r  L %s %s | R %s | %s | %s%s   ' % (left, up, z, trig, trial, feel)


def main():
    if not sys.stdin.isatty():
        sys.exit('run this in an interactive terminal')
    print(__doc__.split('\n\n', 2)[2].rstrip())
    print('\n  Start baseline_direct.py --joy in another terminal, then steer here.\n')

    rclpy.init()
    node = rclpy.create_node('keyboard_joy')
    pub = node.create_publisher(Joy, '/joy', 10)
    feel = {'v': None}

    def on_haptics(msg):
        feel['v'] = (max(0.0, min(1.0, msg.data[0])), max(0.0, min(1.0, msg.data[1])))
    node.create_subscription(Float32MultiArray, '/sc/haptics', on_haptics, 10)
    lx = ly = ry = 0.0
    r2_on, level = False, 1.0

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.02)[0]:
                c = sys.stdin.read(1).lower()
                if c == 'q':
                    break
                elif c == 'a':
                    lx = clamp(lx + STEP)       # joy_node: left is +1
                elif c == 'd':
                    lx = clamp(lx - STEP)
                elif c == 'w':
                    ly = clamp(ly + STEP)
                elif c == 's':
                    ly = clamp(ly - STEP)
                elif c == 'i':
                    ry = clamp(ry + STEP)
                elif c == 'k':
                    ry = clamp(ry - STEP)
                elif c == 'x':
                    lx = ly = ry = 0.0
                elif c == ' ':
                    r2_on = not r2_on
                elif c.isdigit():
                    level = 1.0 if c == '0' else int(c) / 10.0
            # joy_node trigger: +1 released, -1 fully pressed. Exactly 0 means
            # "never touched" to baseline_direct.py, so 50 % is sent as -0.001.
            r2 = 1.0 - 2.0 * level if r2_on else 1.0
            if r2 == 0.0:
                r2 = -0.001
            pub.publish(Joy(axes=[lx, ly, 0.0, 0.0, ry, r2], buttons=[0] * 13))
            rclpy.spin_once(node, timeout_sec=0.0)
            sys.stdout.write(status(lx, ly, ry, r2_on, level, pub.get_subscription_count() > 0,
                                    feel['v']))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # leave the robot still: release R2 and centre the sticks
        for _ in range(5):
            pub.publish(Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0] * 13))
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print('\n  stopped')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
