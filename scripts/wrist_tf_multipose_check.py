#!/usr/bin/env python3
"""Multi-pose check of the measured wrist TF against a known table plane.

The wrist camera sees the same table from several arm poses. For each pose the
script fits the dominant plane in the central depth ROI, transforms it into
``base_link`` with the live ``base_link -> camera_optical_measured`` TF and
reports the plane offset. If the TF chain is correct, the table z stays at the
known reference (-0.0825 m) regardless of pose; drift with pose means a wrong
hand-eye, wrong FK chain or a stale/incorrect joint source.

This is a diagnostic, not a metric estimator: it only needs depth + TF, no
detector and no motion command. A run is meant to be paced by a human moving
the arm between poses while the script logs one row per sample.

Example (inside the robot container):

    python3 wrist_tf_multipose_check.py --duration 180 --out /tmp/multipose.csv
"""

import argparse
import math
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _depth_to_points(message, intrinsics, max_range):
    """Return camera-frame XYZ points for the central ROI of a depth image."""
    height, width = message.height, message.width
    step = message.step
    dtype = np.uint16 if message.encoding in ('16UC1', 'mono16') else np.float32
    raw = np.frombuffer(message.data, dtype=dtype)
    if message.encoding in ('16UC1', 'mono16'):
        rows = step // 2
        image = raw[:rows * height].reshape(height, rows)[:, :width].astype(
            np.float32) * 0.001
    else:
        rows = step // 4
        image = raw[:rows * height].reshape(height, rows)[:, :width]

    fx, fy, cx, cy = intrinsics
    # Central 40 % keeps the arm and table edges out of the fit.
    u0, u1 = int(width * 0.30), int(width * 0.70)
    v0, v1 = int(height * 0.30), int(height * 0.70)
    stride = 4
    us = np.arange(u0, u1, stride)
    vs = np.arange(v0, v1, stride)
    grid_u, grid_v = np.meshgrid(us, vs)
    z = image[grid_v, grid_u]
    valid = np.isfinite(z) & (z > 0.05) & (z < max_range)
    if valid.sum() < 50:
        return None
    z = z[valid]
    x = (grid_u[valid] - cx) / fx * z
    y = (grid_v[valid] - cy) / fy * z
    return np.stack([x, y, z], axis=1)


def _fit_plane(points):
    """Fit a plane with two rounds of outlier rejection; return n, d, rms."""
    keep = points
    for _ in range(2):
        centroid = keep.mean(axis=0)
        _, _, vh = np.linalg.svd(keep - centroid, full_matrices=False)
        normal = vh[-1]
        distances = keep @ normal - centroid @ normal
        sigma = float(np.std(distances))
        if sigma <= 0.0:
            break
        mask = np.abs(distances) < max(2.5 * sigma, 0.002)
        if mask.sum() < 30:
            break
        keep = keep[mask]
    normal = normal / np.linalg.norm(normal)
    d = float(normal @ centroid)
    rms = float(np.sqrt(np.mean((keep @ normal - d) ** 2)))
    return normal, d, rms, int(keep.shape[0])


class MultiposeCheck(Node):
    """Log the table plane in base_link for each wrist-camera pose."""

    def __init__(self, args):
        super().__init__('wrist_tf_multipose_check')
        self._args = args
        self._depth = None
        self._intrinsics = None
        self._buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._listener = TransformListener(self._buffer, self)
        self.create_subscription(
            Image, args.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, args.info_topic, self._on_info, qos_profile_sensor_data)
        self._out = open(args.out, 'w', encoding='utf-8')
        self._out.write(
            't_ns,tf_ok,cam_x,cam_y,cam_z,qx,qy,qz,qw,'
            'n_x,n_y,n_z,table_z,margin_m,rms_m,points\n')
        self._samples = 0
        self._tf_failures = 0

    def _on_depth(self, message):
        self._depth = message

    def _on_info(self, message):
        k = message.k
        self._intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

    def sample(self):
        """Fit the plane once and append a CSV row; return a summary tuple."""
        if self._depth is None or self._intrinsics is None:
            return None
        points = _depth_to_points(
            self._depth, self._intrinsics, self._args.max_range)
        if points is None:
            return None
        normal, d, rms, count = _fit_plane(points)
        if abs(d) < 0.08:
            # The fitted plane is too close to the camera to be the table;
            # it is almost certainly the arm or the gripper in the ROI.
            return None
        try:
            transform = self._buffer.lookup_transform(
                self._args.base_frame, self._args.camera_frame,
                rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF lookup failures are expected
            self._tf_failures += 1
            self._out.write('{},{}\n'.format(
                self.get_clock().now().nanoseconds, 0))
            self._out.flush()
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        matrix = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        origin = np.array([translation.x, translation.y, translation.z])
        n_base = matrix @ normal
        d_base = d + float(n_base @ origin)
        if n_base[2] < 0.0:
            n_base = -n_base
            d_base = -d_base
        table_z = d_base
        tilt_deg = math.degrees(math.acos(min(1.0, abs(n_base[2]))))
        row = (
            self.get_clock().now().nanoseconds, 1,
            translation.x, translation.y, translation.z,
            x, y, z, w,
            n_base[0], n_base[1], n_base[2],
            table_z, origin[2] - table_z, rms, count,
        )
        self._out.write(','.join(
            [str(row[0]), str(row[1])] +
            ['{:.6f}'.format(v) for v in row[2:13]] +
            ['{:.6f}'.format(row[13]), '{:.6f}'.format(row[14]),
             str(row[15])]) + '\n')
        self._out.flush()
        self._samples += 1
        return table_z, tilt_deg, rms, count, origin[2] - table_z

    def close(self):
        """Flush and close the CSV."""
        self._out.close()


def main(argv=None):
    """Run the sampling loop until the duration expires or SIGINT arrives."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--depth-topic', default='/depth_cam/depth/image_raw')
    parser.add_argument('--info-topic', default='/depth_cam/rgb/camera_info')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--camera-frame', default='camera_optical_measured')
    parser.add_argument('--duration', type=float, default=180.0)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--max-range', type=float, default=1.0)
    parser.add_argument('--out', default='/tmp/wrist_multipose.csv')
    args = parser.parse_args(argv)

    rclpy.init()
    node = MultiposeCheck(args)
    stop = {'flag': False}

    def _handle_sigint(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, _handle_sigint)

    deadline = time.time() + args.duration
    print('sampling {} -> {} at {:.1f} Hz for {:.0f} s'.format(
        args.depth_topic, args.camera_frame, args.rate, args.duration),
        flush=True)
    try:
        while not stop['flag'] and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            result = node.sample()
            if result is not None:
                table_z, tilt_deg, rms, count, margin = result
                print(
                    'table_z={:+.4f} m  tilt={:.2f} deg  rms={:.4f} m  '
                    'points={}  cam_height={:.3f} m'.format(
                        table_z, tilt_deg, rms, count, margin), flush=True)
            time.sleep(1.0 / args.rate)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
