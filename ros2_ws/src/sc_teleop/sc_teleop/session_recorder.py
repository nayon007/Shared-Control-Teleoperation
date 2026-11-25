"""Record one driving session: video with live numbers, full data log, summary.

    ros2 run sc_teleop session_recorder --ros-args -p out_dir:=/path -p mode:=assisted

Started by teleop.launch.py (record:=true, the default for keyboard and
DualSense). Writes into out_dir:
  video.mp4     scene camera (15 fps) with mode, friction, opening, off-axis
                force, rumble, trigger and stick input drawn on top
  data.csv      /sc/status at 100 Hz (shared_control.STATUS_FIELDS)
  summary.json  settings and results: furthest opening, time to the target,
                peak and mean off-axis force, time above the handle-loss
                limit, how often the limit was crossed
Everything is written when the session ends (Ctrl+C).
"""

import json
import os
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from sc_teleop.shared_control import STATUS_FIELDS

FPS = 15.0


def bar(img, x, y, w, h, frac, color, label, value):
    import cv2
    cv2.rectangle(img, (x, y), (x + w, y + h), (70, 70, 70), -1)
    cv2.rectangle(img, (x, y), (x + int(w * max(0.0, min(1.0, frac))), y + h), color, -1)
    cv2.putText(img, label, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1,
                cv2.LINE_AA)
    cv2.putText(img, value, (x + w + 8, y + h - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)


class Recorder(Node):

    def __init__(self):
        super().__init__('session_recorder')
        for name, default in (('out_dir', ''), ('mode', ''), ('kind', ''), ('friction', 0.0),
                              ('damping', 0.0), ('yaw', 0.0), ('input', ''),
                              ('slip_force', 30.0), ('target', 0.25), ('video', True)):
            self.declare_parameter(name, default)
        self.p = {k: self.get_parameter(k).value for k in (
            'out_dir', 'mode', 'kind', 'friction', 'damping', 'yaw', 'input', 'slip_force',
            'target', 'video')}
        os.makedirs(self.p['out_dir'], exist_ok=True)
        self.rows, self.last, self.writer, self.frames = [], None, None, 0
        self.started = time.strftime('%Y-%m-%d %H:%M:%S')
        self.create_subscription(Float32MultiArray, '/sc/status', self._status, 100)
        if self.p['video']:
            self.create_subscription(Image, '/scene_camera/image_raw', self._image, 5)
        self.get_logger().info('recording this session into %s' % self.p['out_dir'])

    def _status(self, m):
        self.rows.append(list(m.data))
        self.last = list(m.data)

    def _image(self, m):
        import cv2
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        img = cv2.cvtColor(a.reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)
        self.overlay(img)
        if self.writer is None:
            path = os.path.join(self.p['out_dir'], 'video.mp4')
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), FPS,
                                          (m.width, m.height))
        self.writer.write(img)
        self.frames += 1

    def overlay(self, img):
        import cv2
        P = self.p
        cv2.rectangle(img, (0, 0), (img.shape[1], 70), (30, 30, 30), -1)
        title = '%s control   %s, friction %g N   %s' % (
            P['mode'].upper(), P['kind'], P['friction'], P['input'])
        cv2.putText(img, title, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
                    cv2.LINE_AA)
        if self.last is None:
            cv2.putText(img, 'waiting for shared_control ...', (16, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
            return
        s = dict(zip(STATUS_FIELDS, self.last))
        cv2.putText(img, 't %.1f s   confidence c %.2f' % (s['t'], s['c']), (16, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        h = img.shape[0]
        lost = s['F_perp'] > P['slip_force']
        x0, y0 = 16, h - 150
        cv2.rectangle(img, (0, y0 - 30), (520, h), (30, 30, 30), -1)
        bar(img, x0, y0, 330, 16, s['opening'] / max(P['target'], 1e-6), (80, 200, 80),
            'opening', '%.3f m' % s['opening'])
        bar(img, x0, y0 + 40, 330, 16, s['F_perp'] / 60.0,
            (40, 40, 230) if lost else (60, 170, 240), 'off-axis force (limit 30 N)',
            '%.1f N' % s['F_perp'])
        lim = x0 + int(330 * P['slip_force'] / 60.0)
        cv2.line(img, (lim, y0 + 36), (lim, y0 + 60), (255, 255, 255), 2)
        bar(img, x0, y0 + 80, 150, 12, s['rumble'], (200, 120, 240), 'rumble', '%.2f' % s['rumble'])
        bar(img, x0 + 250, y0 + 80, 150, 12, s['trigger'], (240, 180, 60), 'R2 resistance',
            '%.2f' % s['trigger'])
        speed = np.linalg.norm([s['u_x'], s['u_y'], s['u_z']])
        cv2.putText(img, 'operator speed %.3f m/s' % speed, (x0, y0 + 122),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        if lost:
            cv2.putText(img, 'HANDLE WOULD BE LOST', (img.shape[1] // 2 - 230, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (40, 40, 240), 3, cv2.LINE_AA)

    def finish(self):
        if self.writer is not None:
            self.writer.release()
        out = self.p['out_dir']
        meta = dict(self.p, started=self.started, video_frames=self.frames)
        if self.rows:
            d = np.asarray(self.rows, dtype=float)
            np.savetxt(os.path.join(out, 'data.csv'), d, delimiter=',',
                       header=','.join(STATUS_FIELDS), comments='', fmt='%.5g')
            col = {k: d[:, i] for i, k in enumerate(STATUS_FIELDS)}
            t, fp = col['t'], col['F_perp']
            dt = np.diff(t, prepend=t[0])
            above = fp > self.p['slip_force']
            reached = np.nonzero(col['opening'] >= self.p['target'])[0]
            moving = np.linalg.norm(np.column_stack([col['u_x'], col['u_y'], col['u_z']]),
                                    axis=1) > 1e-3
            meta.update(
                duration_s=float(t[-1]),
                max_opening_m=float(col['opening'].max()),
                time_to_target_s=float(t[reached[0]]) if reached.size else None,
                peak_F_perp_N=float(fp.max()),
                mean_F_perp_while_moving_N=float(fp[moving].mean()) if moving.any() else 0.0,
                time_above_limit_s=float(dt[above].sum()),
                limit_crossings=int(np.sum(above[1:] & ~above[:-1]) + above[0]),
                peak_F_par_N=float(np.abs(col['F_par']).max()),
                mean_rumble_while_moving=float(col['rumble'][moving].mean()) if moving.any()
                else 0.0,
            )
        with open(os.path.join(out, 'summary.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        print('session saved: %s (%d data rows, %d video frames)'
              % (out, len(self.rows), self.frames), flush=True)


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
