"""Validate the measured wrist TF by fitting the table plane in base_link.

Reads one depth frame and the depth CameraInfo, transforms sampled pixels with
base_link -> camera_optical_measured (wrist_tf), and reports the fitted plane.
No motion commands; expects the arm to be stationary.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


class PlaneCheck(Node):
    def __init__(self):
        super().__init__('wrist_plane_check')
        self.bridge = CvBridge()
        self.depth = None
        self.info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            Image, '/depth_cam/depth/image_raw', self.on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/depth_cam/depth/camera_info', self.on_info,
            qos_profile_sensor_data)

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message


def transform_matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
         2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
         2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x),
         1 - 2 * (x * x + y * y)],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [transform.translation.x,
                     transform.translation.y,
                     transform.translation.z]
    return matrix


def fit_plane(points):
    rng = np.random.default_rng(0)
    best = None
    for _ in range(300):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        distance = np.abs(points @ normal + offset)
        inliers = distance < 0.01
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, normal, offset, inliers)
    count, normal, offset, inliers = best
    floor = points[inliers]
    center = floor.mean(axis=0)
    _, _, vt = np.linalg.svd(floor - center)
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    offset = -float(normal @ center)
    residual = floor @ normal + offset
    return normal, offset, inliers, residual


def main():
    rclpy.init()
    node = PlaneCheck()
    deadline = node.get_clock().now().nanoseconds + 20_000_000_000
    while rclpy.ok() and (node.depth is None or node.info is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            break
    if node.depth is None or node.info is None:
        print('missing depth or camera_info')
        node.destroy_node()
        rclpy.shutdown()
        return
    # Let the TF listener catch up before the first lookup.
    tf_deadline = node.get_clock().now().nanoseconds + 10_000_000_000
    latest_transform = None
    while rclpy.ok():
        try:
            latest_transform = node.buffer.lookup_transform(
                'base_link', 'camera_optical_measured', Time())
            break
        except Exception:  # noqa: BLE001 - wait for the listener
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > tf_deadline:
                break
    if latest_transform is None:
        print('camera_optical_measured TF did not arrive')
        node.destroy_node()
        rclpy.shutdown()
        return
    depth = node.bridge.imgmsg_to_cv2(
        node.depth, desired_encoding='passthrough')
    k = np.asarray(node.info.k, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    stamp = Time.from_msg(node.depth.header.stamp)
    try:
        transform = node.buffer.lookup_transform(
            'base_link', 'camera_optical_measured', stamp)
        used = 'depth stamp'
    except Exception as error:  # noqa: BLE001 - report the fallback
        print('exact-stamp lookup failed: {}'.format(error))
        transform = node.buffer.lookup_transform(
            'base_link', 'camera_optical_measured', Time())
        used = 'latest'
    matrix = transform_matrix(transform.transform)
    translation = matrix[:3, 3]
    view = matrix[:3, 2]
    print('tf_frame source:', used)
    print('camera_position_m:', np.round(translation, 4).tolist())
    print('camera_view_z:', np.round(view, 4).tolist())
    print('camera_height_m:', round(float(translation[2]), 4))
    print('depth_stamp:', node.depth.header.stamp.sec,
          node.depth.header.stamp.nanosec)
    print('depth_frame:', node.depth.header.frame_id)
    print('depth_shape:', depth.shape, 'K:', np.round(k, 2).tolist())

    rows = np.arange(40, 340, 6)
    cols = np.arange(40, 600, 6)
    uu, vv = np.meshgrid(cols, rows)
    zz = depth[vv, uu].astype(np.float64) * 0.001
    valid = (zz > 0.10) & (zz < 1.20)
    uu = uu[valid].astype(np.float64)
    vv = vv[valid].astype(np.float64)
    zz = zz[valid]
    camera_points = np.stack([
        (uu - cx) * zz / fx,
        (vv - cy) * zz / fy,
        zz,
    ], axis=1)
    base_points = (matrix[:3, :3] @ camera_points.T).T + translation
    print('valid_samples:', len(base_points),
          'base_z_range:', round(float(base_points[:, 2].min()), 3),
          round(float(base_points[:, 2].max()), 3))
    if len(base_points) < 50:
        print('too few valid points')
        node.destroy_node()
        rclpy.shutdown()
        return
    normal, offset, inliers, residual = fit_plane(base_points)
    inlier_fraction = float(inliers.mean())
    plane_z_at_origin = -offset / normal[2]
    tilt_deg = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
    print('plane_normal:', np.round(normal, 4).tolist())
    print('plane_z_at_base_origin_m:',
          round(float(plane_z_at_origin), 4))
    print('plane_tilt_deg:', round(tilt_deg, 2))
    print('inlier_fraction:', round(inlier_fraction, 3))
    print('plane_rms_mm:', round(float(residual.std() * 1000.0), 2))
    print('camera_above_plane_m:',
          round(float(translation[2] - plane_z_at_origin), 4))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
