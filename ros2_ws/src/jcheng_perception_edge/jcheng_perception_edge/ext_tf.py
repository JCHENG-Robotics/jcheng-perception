"""Publish ``base_link -> camera_link`` for the chassis D435i.

Before ADR 0004 the vendor ``ext_cam_node`` owned the UVC colour stream and
published ``base_link -> ext_cam_optical`` itself. With the single-owner
RealSense node that static link is gone: the driver's own tree
(``camera_link -> camera_color_optical_frame``) floats disconnected from
``base_link``. The edge node still back-projects with the measured fallback in
``config/frames.yaml``, but RViz, the studio gateway and any other TF consumer
then see two unconnected trees.

``T_base_front`` maps *colour optical* coordinates to ``base_link``
(2026-11-20 chessboard result). The RealSense driver supplies the fixed
link/optical rotation, so this node only has to publish one static transform:

    T_base_camera_link = T_base_optical * inverse(T_link_optical)

with ``T_link_optical`` the standard body->optical rotation (x forward, y left,
z up  ->  x right, y down, z forward). Nothing here moves with the arm; a
missing or non-orthonormal extrinsic is a fatal startup error so the node fails
closed instead of publishing wrong geometry.
"""

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.logging import get_logger as get_rclpy_logger
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from jcheng_perception_core.geometry import (
    matrix_from_quaternion,
    quaternion_from_matrix,
)

# Camera body frame (ROS: x forward, y left, z up) -> optical frame
# (x right, y down, z forward). p_link = R_LINK_OPTICAL @ p_optical.
R_LINK_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)


def _as_matrix(value):
    """Validate and return a 4x4 homogeneous transform."""
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('extrinsic must be 4x4, got {}'.format(matrix.shape))
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError('extrinsic bottom row must be [0, 0, 0, 1]')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError('extrinsic rotation is not orthonormal')
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6:
        raise ValueError('extrinsic rotation determinant is not +1')
    if not np.all(np.isfinite(matrix)):
        raise ValueError('extrinsic contains non-finite values')
    return matrix


def base_camera_link_transform(t_base_optical, t_link_optical=None):
    """Compose the base -> camera_link transform from the optical extrinsic."""
    if t_link_optical is None:
        t_link_optical = np.eye(4)
        t_link_optical[:3, :3] = R_LINK_OPTICAL
    return _as_matrix(np.asarray(t_base_optical, dtype=np.float64)) @ \
        np.linalg.inv(_as_matrix(t_link_optical))


def _load_extrinsic(frames_path, camera_id):
    """Read ``static_fallbacks.<camera_id>.transform`` from frames.yaml."""
    with open(frames_path, 'r', encoding='utf-8') as stream:
        frames = yaml.safe_load(stream)
    if not isinstance(frames, dict):
        raise ValueError('frames config must be a mapping')
    fallbacks = frames.get('static_fallbacks') or {}
    entry = fallbacks.get(camera_id)
    if not isinstance(entry, dict) or 'transform' not in entry:
        raise ValueError(
            'frames config has no static_fallbacks.{} transform'.format(
                camera_id))
    return _as_matrix(entry['transform'])


class ExtTfNode(Node):
    """Publish one static ``base_link -> camera_link`` transform."""

    def __init__(self):
        """Load the extrinsic and broadcast the composed transform."""
        super().__init__('ext_tf')
        self.declare_parameter('frames_config', '')
        self.declare_parameter('config_file', '')
        frames_path = self.get_parameter('frames_config').value
        config_path = self.get_parameter('config_file').value
        if not frames_path:
            raise RuntimeError('frames_config parameter is required')
        if not config_path:
            raise RuntimeError('config_file parameter is required')
        with open(config_path, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        camera_id = str(config.get('camera_id', 'd435i_ext'))
        base_frame = str(config.get('base_frame', 'base_link'))
        camera_frame = str(config.get('camera_link_frame', 'camera_link'))
        optical_frame = str(
            config.get('optical_frame', 'camera_color_optical_frame'))

        t_base_optical = _load_extrinsic(frames_path, camera_id)
        self._t_base_optical = t_base_optical
        self._base_frame = base_frame
        self._camera_frame = camera_frame
        self._optical_frame = optical_frame
        self._broadcaster = StaticTransformBroadcaster(self)
        self._published = False
        self._attempts = 0
        # The driver's own camera_link -> colour-optical transform carries the
        # 15 mm colour/depth baseline and its calibration, so wait for it and
        # invert it instead of assuming the ideal rotation.
        self._buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._listener = TransformListener(self._buffer, self)
        self.create_timer(0.5, self._try_publish)

    def _try_publish(self):
        """Publish once the driver transform is known (or after a timeout)."""
        if self._published:
            return
        self._attempts += 1
        t_link_optical = None
        try:
            transform = self._buffer.lookup_transform(
                self._camera_frame, self._optical_frame, Time())
        except Exception:  # noqa: BLE001 - wait for the static TF
            if self._attempts < 10:
                return
            self.get_logger().warn(
                'ext_tf: {} -> {} not available after 5 s; using the ideal '
                'link/optical rotation'.format(
                    self._camera_frame, self._optical_frame))
        else:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            t_link_optical = np.eye(4)
            t_link_optical[:3, :3] = matrix_from_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w))
            t_link_optical[:3, 3] = [
                translation.x, translation.y, translation.z]

        t_base_camera = base_camera_link_transform(
            self._t_base_optical, t_link_optical)
        self._publish(t_base_camera)
        self._published = True

    def _publish(self, t_base_camera):
        """Broadcast the composed static transform and log the result."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._camera_frame
        transform.transform.translation.x = float(t_base_camera[0, 3])
        transform.transform.translation.y = float(t_base_camera[1, 3])
        transform.transform.translation.z = float(t_base_camera[2, 3])
        quaternion = quaternion_from_matrix(t_base_camera[:3, :3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._broadcaster.sendTransform(transform)

        # view_z is the optical axis expressed in base_link; for the chassis
        # camera it must point forward and down (pitch 32.6 deg).
        view_z = self._t_base_optical[:3, 2]
        self.get_logger().info(
            'ext_tf: {} -> {} at ({:+.3f}, {:+.3f}, {:+.3f}) m, '
            '{} view_z=({:+.3f}, {:+.3f}, {:+.3f})'.format(
                self._base_frame, self._camera_frame,
                t_base_camera[0, 3], t_base_camera[1, 3],
                t_base_camera[2, 3], self._optical_frame,
                view_z[0], view_z[1], view_z[2]))


def main(args=None):
    """Run the static external-camera TF publisher."""
    rclpy.init(args=args)
    try:
        node = ExtTfNode()
    except Exception as error:  # noqa: BLE001 - fatal startup error
        get_rclpy_logger('ext_tf').fatal(
            'ext_tf failed to start: {}'.format(error))
        rclpy.shutdown()
        raise
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
