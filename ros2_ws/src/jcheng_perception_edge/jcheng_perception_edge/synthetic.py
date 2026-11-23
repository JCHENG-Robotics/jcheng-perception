"""
Synthetic two-camera scene for offline end-to-end tests.

It publishes the same topics the real cameras publish, with a red square on a
table plane. The mock GPU detector thresholds the red square, so the complete
distributed path (edge compression -> GPU "inference" -> edge time matching ->
depth back-projection -> fusion) can be exercised on one machine without a
model or hardware.

The external camera uses the measured chassis extrinsic and the table height
from the 2026-11-20 calibration (table z = -0.0825 m in base_link).
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from jcheng_perception_core import quaternion_from_matrix

TABLE_Z = -0.0825
TARGET_SIZE = 0.06
TARGET_CENTER = np.array([0.35, 0.02, TABLE_Z])
# Measured chassis D435i optical frame in base_link (2026-11-20).
EXT_TRANSLATION = np.array([0.1393005143144151, 0.027041016800109785,
                            0.03674937694821427])
EXT_ROTATION = np.array([
    [-0.04216838436036936, -0.5377799793511505, 0.8420300001600372],
    [-0.9986357296162156, 0.04866561522702682, -0.018929802652546318],
    [-0.030797839117794233, -0.8416794827626792, -0.5390984524203566],
])
# A downward-looking wrist camera with x along base x and y along -base y.
WRIST_TRANSLATION = np.array([0.35, 0.02, 0.28])
WRIST_ROTATION = np.diag([1.0, -1.0, -1.0])


def _sensor_qos():
    """Best-effort camera QoS."""
    return QoSProfile(
        depth=2,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _latched_qos():
    """Transient-local CameraInfo QoS."""
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def camera_transform(translation, rotation):
    """Return a 4x4 base<-camera transform."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def target_corners(center, yaw, size=TARGET_SIZE):
    """Return the four table-plane corners of the red square."""
    half = size / 2.0
    local = np.array([
        [-half, -half], [half, -half], [half, half], [-half, half]
    ])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    corners = local @ rotation.T
    return np.column_stack([
        corners[:, 0] + center[0],
        corners[:, 1] + center[1],
        np.full(4, center[2]),
    ])


def project(camera_matrix, transform_base_camera, points_base):
    """Project base-frame points into an image."""
    points = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    rotation = transform_base_camera[:3, :3]
    translation = transform_base_camera[:3, 3]
    points_camera = (points - translation) @ rotation
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    z = points_camera[:, 2]
    safe_z = np.where(np.abs(z) < 1e-6, 1e-6, z)
    u = intrinsics[0, 0] * points_camera[:, 0] / safe_z + intrinsics[0, 2]
    v = intrinsics[1, 1] * points_camera[:, 1] / safe_z + intrinsics[1, 2]
    return np.column_stack([u, v]), z


def render_plane_depth(camera_matrix, transform_base_camera, width, height):
    """Render the static table plane as a uint16 depth image (millimetres)."""
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    columns, rows = np.meshgrid(
        np.arange(width, dtype=np.float64),
        np.arange(height, dtype=np.float64),
    )
    x = (columns - cx) / fx
    y = (rows - cy) / fy
    rays_camera = np.stack(
        [x, y, np.ones_like(x)], axis=-1).reshape(-1, 3)
    rotation = transform_base_camera[:3, :3]
    translation = transform_base_camera[:3, 3]
    rays_base = rays_camera @ rotation.T
    denominator = rays_base[:, 2]
    valid = np.abs(denominator) > 1e-6
    scale = np.full(denominator.shape, -1.0, dtype=np.float64)
    scale[valid] = (TABLE_Z - translation[2]) / denominator[valid]
    depth = np.where(valid & (scale > 0.0), scale, 0.0)
    depth = depth.reshape(height, width)
    return np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)


def render(camera_matrix, transform_base_camera, width, height,
           target_center, target_yaw, depth_noise_m=0.0, rng=None,
           plane_depth_mm=None):
    """Render BGR and uint16 depth for one camera."""
    if plane_depth_mm is None:
        depth_mm = render_plane_depth(
            camera_matrix, transform_base_camera, width, height)
    else:
        depth_mm = np.array(plane_depth_mm, copy=True)
    if depth_noise_m > 0.0 and rng is not None:
        noise = rng.normal(
            0.0, depth_noise_m * 1000.0, size=depth_mm.shape)
        positive = depth_mm > 0
        depth_mm[positive] = np.clip(
            depth_mm[positive] + noise[positive], 0.0, 65535.0).astype(
                np.uint16)

    image = np.full((height, width, 3), 55, dtype=np.uint8)
    corners = target_corners(
        np.asarray(target_center, dtype=np.float64), target_yaw)
    pixels, z_values = project(
        camera_matrix, transform_base_camera, corners)
    if np.all(z_values > 0.0):
        polygon = np.round(pixels).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(image, [polygon], (0, 0, 235))
        cv2.polylines(
            image, [polygon], True, (0, 180, 0), 1, cv2.LINE_AA)
    return image, depth_mm


def camera_info(camera_matrix, width, height, frame_id):
    """Build a plumb-bob CameraInfo message."""
    message = CameraInfo()
    message.width = int(width)
    message.height = int(height)
    message.distortion_model = 'plumb_bob'
    message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    message.k = [float(value) for value in matrix.reshape(9)]
    message.p = [
        float(matrix[0, 0]), 0.0, float(matrix[0, 2]), 0.0,
        0.0, float(matrix[1, 1]), float(matrix[1, 2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    message.header.frame_id = frame_id
    return message


class SyntheticScene(Node):
    """Publish two synthetic camera streams, CameraInfo and TF."""

    def __init__(self):
        """Create publishers and the render timer."""
        super().__init__('synthetic_scene')
        cv2.setNumThreads(1)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('depth_noise_m', 0.0008)
        self.declare_parameter('focal_px', 500.0)
        self.declare_parameter('seed', 7)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.depth_noise = float(
            self.get_parameter('depth_noise_m').value)
        focal = float(self.get_parameter('focal_px').value)
        self.rng = np.random.default_rng(
            int(self.get_parameter('seed').value))
        self.camera_matrix = np.array([
            [focal, 0.0, self.width / 2.0],
            [0.0, focal, self.height / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.ext_transform = camera_transform(
            EXT_TRANSLATION, EXT_ROTATION)
        self.wrist_transform = camera_transform(
            WRIST_TRANSLATION, WRIST_ROTATION)
        self.ext_frame = 'synthetic_ext_optical'
        self.wrist_frame = 'synthetic_wrist_optical'
        self.plane_depth = {
            self.ext_frame: render_plane_depth(
                self.camera_matrix, self.ext_transform,
                self.width, self.height),
            self.wrist_frame: render_plane_depth(
                self.camera_matrix, self.wrist_transform,
                self.width, self.height),
        }
        self.bridge = CvBridge()
        self.ext_rgb_publisher = self.create_publisher(
            Image, '/synthetic/ext/image_raw', _sensor_qos())
        self.ext_depth_publisher = self.create_publisher(
            Image, '/synthetic/ext/depth', _sensor_qos())
        self.wrist_rgb_publisher = self.create_publisher(
            Image, '/synthetic/wrist/image_raw', _sensor_qos())
        self.wrist_depth_publisher = self.create_publisher(
            Image, '/synthetic/wrist/depth', _sensor_qos())
        self.ext_info_publisher = self.create_publisher(
            CameraInfo, '/synthetic/ext/camera_info', _latched_qos())
        self.wrist_info_publisher = self.create_publisher(
            CameraInfo, '/synthetic/wrist/camera_info', _latched_qos())
        self.tf_broadcaster = TransformBroadcaster(self)
        self._start_ns = self.get_clock().now().nanoseconds
        self._publish_camera_info()
        self.create_timer(1.0 / max(1.0, self.rate_hz), self.on_timer)
        self.get_logger().info(
            'synthetic scene: {}x{} at {:.1f} Hz'.format(
                self.width, self.height, self.rate_hz))

    def _publish_camera_info(self):
        """Publish latched intrinsics for both cameras."""
        stamp = self.get_clock().now().to_msg()
        ext = camera_info(
            self.camera_matrix, self.width, self.height, self.ext_frame)
        wrist = camera_info(
            self.camera_matrix, self.width, self.height, self.wrist_frame)
        ext.header.stamp = stamp
        wrist.header.stamp = stamp
        self.ext_info_publisher.publish(ext)
        self.wrist_info_publisher.publish(wrist)

    def _broadcast(self, transform, child_frame, stamp):
        """Publish one base_link -> optical transform."""
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        message.child_frame_id = child_frame
        translation = transform[:3, 3]
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        quaternion = quaternion_from_matrix(transform[:3, :3])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(message)

    def on_timer(self):
        """Render the current target pose and publish all streams."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        elapsed = (now.nanoseconds - self._start_ns) / 1e9
        target = np.array([
            TARGET_CENTER[0] + 0.035 * math.sin(0.35 * elapsed),
            TARGET_CENTER[1] + 0.025 * math.sin(0.22 * elapsed + 0.5),
            TABLE_Z,
        ])
        yaw = 0.4 * math.sin(0.15 * elapsed)
        for transform, frame, rgb_publisher, depth_publisher in (
            (self.ext_transform, self.ext_frame,
             self.ext_rgb_publisher, self.ext_depth_publisher),
            (self.wrist_transform, self.wrist_frame,
             self.wrist_rgb_publisher, self.wrist_depth_publisher),
        ):
            image, depth = render(
                self.camera_matrix, transform, self.width, self.height,
                target, yaw, depth_noise_m=self.depth_noise, rng=self.rng,
                plane_depth_mm=self.plane_depth[frame])
            rgb_message = self.bridge.cv2_to_imgmsg(
                image, encoding='bgr8')
            rgb_message.header.stamp = stamp
            rgb_message.header.frame_id = frame
            depth_message = self.bridge.cv2_to_imgmsg(
                depth, encoding='16UC1')
            depth_message.header.stamp = stamp
            depth_message.header.frame_id = frame
            rgb_publisher.publish(rgb_message)
            depth_publisher.publish(depth_message)
            self._broadcast(transform, frame, stamp)


def main(args=None):
    """Run the synthetic scene publisher."""
    rclpy.init(args=args)
    node = SyntheticScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
