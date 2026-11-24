"""Publish the wrist camera optical frame from measured bus-servo joints.

Why this node exists: the vendor ``robot_state_publisher`` consumes
``/joint_states``, which is the controller's *command cache*, not encoder
feedback.  The cache can stay at a stale pose (measured: joint1 +0.838 rad,
joints 2-5 zero) while the bus servos are somewhere else (joint2 +0.406,
joint3 -1.382, joint4 -1.629 rad).  Composing the stock TF with the calibrated
hand-eye then puts the camera at the wrong pose and the target far above the
table.

This node reads the bus positions through the vendor service, rebuilds
``base_link -> link4`` from the live ``/robot_description`` with forward
kinematics, composes the validated ``T_link4_camera`` and publishes a unique
dynamic frame, by default ``camera_optical_measured``.  It publishes nothing
when a read fails, so the edge fails closed instead of using stale geometry.

Runtime dependency: ``ros_robot_controller_msgs`` (vendor workspace), available
in the robot container.  The import is guarded so the package still builds and
imports on a plain ROS host.
"""

import signal
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.logging import get_logger as get_rclpy_logger
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from jcheng_perception_core.geometry import quaternion_from_matrix

from .wrist_kinematics import (
    forward_kinematics,
    parse_chain,
    pulses_to_positions,
)

try:  # vendor package; present in the robot container, absent in CI
    from ros_robot_controller_msgs.msg import GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState
except ImportError:  # pragma: no cover - exercised only off-robot
    GetBusServoCmd = None
    GetBusServoState = None

REQUIRED_KEYS = (
    'base_frame',
    'tip_frame',
    'camera_frame',
    'servo_service',
    'servo_ids',
    'link4_camera',
)


def _as_matrix(value):
    """Validate and return a 4x4 homogeneous transform."""
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('link4_camera must be 4x4, got {}'.format(
            matrix.shape))
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError('link4_camera bottom row must be [0, 0, 0, 1]')
    rotation = matrix[:3, :3]
    if not np.allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError('link4_camera rotation is not orthonormal')
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6:
        raise ValueError('link4_camera rotation determinant is not +1')
    return matrix


def _load_config(path):
    """Read and validate the wrist TF configuration."""
    with open(path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('wrist TF config must be a mapping')
    missing = [key for key in REQUIRED_KEYS if key not in config]
    if missing:
        raise ValueError(
            'wrist TF config missing keys: {}'.format(', '.join(missing)))
    rate = float(config.get('update_rate_hz', 10.0))
    if not 1.0 <= rate <= 30.0:
        raise ValueError('update_rate_hz must be within 1..30')
    if not isinstance(config['servo_ids'], dict):
        raise ValueError('servo_ids must map joint names to servo ids')
    config['link4_camera'] = _as_matrix(config['link4_camera'])
    config['update_rate_hz'] = rate
    config['center_pulse'] = int(config.get('center_pulse', 500))
    config['radians_per_tick'] = float(config.get('radians_per_tick', 0.0))
    if config['radians_per_tick'] <= 0.0:
        raise ValueError('radians_per_tick must be positive')
    inverted = config.get('inverted_joints', [])
    if not isinstance(inverted, (list, tuple)):
        raise ValueError('inverted_joints must be a list')
    config['inverted_joints'] = tuple(inverted)
    return config


class WristTfNode(Node):
    """Publish ``base_link -> camera_optical_measured`` at the bus read rate."""

    def __init__(self):
        """Load the configuration and start the bus polling timer."""
        super().__init__('wrist_tf')
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        if not config_path:
            raise RuntimeError('config_file parameter is required')
        config = _load_config(config_path)
        self._base_frame = str(config['base_frame'])
        self._tip_frame = str(config['tip_frame'])
        self._camera_frame = str(config['camera_frame'])
        self._hand_eye = config['link4_camera']
        self._service = str(config['servo_service'])
        self._servo_ids = {
            str(name): int(servo_id)
            for name, servo_id in config['servo_ids'].items()
        }
        self._servo_id_order = list(self._servo_ids.values())
        self._center_pulse = config['center_pulse']
        self._radians_per_tick = config['radians_per_tick']
        self._inverted_joints = config['inverted_joints']
        self._rate = config['update_rate_hz']

        self._chain = None
        self._pending = None
        self._last_success = 0.0
        self._last_log = 0.0
        self._last_warn = 0.0
        self._reads = 0
        self._failures = 0

        if GetBusServoState is None or GetBusServoCmd is None:
            self.get_logger().fatal(
                'ros_robot_controller_msgs is not available; source the '
                'vendor workspace before starting wrist_tf')
            raise RuntimeError('ros_robot_controller_msgs is missing')

        self._broadcaster = TransformBroadcaster(self)
        description_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/robot_description', self._on_description,
            description_qos)
        self._client = self.create_client(GetBusServoState, self._service)
        self.create_timer(1.0 / self._rate, self._on_timer)
        self.get_logger().info(
            'wrist_tf: {} -> {} at {:.0f} Hz, service {}'.format(
                self._base_frame, self._camera_frame, self._rate,
                self._service))

    # ------------------------------------------------------------- URDF
    def _on_description(self, message):
        """Parse the kinematic chain once the robot description arrives."""
        try:
            self._chain = parse_chain(
                message.data, self._base_frame, self._tip_frame)
        except ValueError as error:
            self.get_logger().error(
                'cannot parse wrist TF chain: {}'.format(error))
            return
        joints = [joint['name'] for joint in self._chain]
        self.get_logger().info(
            'wrist_tf chain {} -> {}: {}'.format(
                self._base_frame, self._tip_frame, ' -> '.join(joints)))

    # ------------------------------------------------------------ servo
    def _on_timer(self):
        """Read the bus at the configured rate and publish the new pose."""
        if self._chain is None:
            return
        if self._pending is not None:
            if not self._pending.done():
                return
            future = self._pending
            self._pending = None
            try:
                response = future.result()
            except Exception as error:  # noqa: BLE001 - service is vendor
                self._fail('bus servo read raised: {}'.format(error))
            else:
                self._handle_response(response)
        if not self._client.service_is_ready():
            self._fail('bus servo service not available: {}'.format(
                self._service))
            return
        request = GetBusServoState.Request()
        for servo_id in self._servo_id_order:
            command = GetBusServoCmd()
            command.id = int(servo_id)
            command.get_position = 1
            request.cmd.append(command)
        self._pending = self._client.call_async(request)

    def _handle_response(self, response):
        """Convert the response to a camera pose and publish the TF."""
        if response is None or not response.success:
            self._fail('bus servo read returned failure')
            return
        if len(response.state) != len(self._servo_id_order):
            self._fail(
                'bus servo response has {} states for {} requests'.format(
                    len(response.state), len(self._servo_id_order)))
            return
        pulses = {}
        for servo_id, state in zip(self._servo_id_order, response.state):
            if not state.position:
                self._fail(
                    'bus servo {} returned no position'.format(servo_id))
                return
            pulses[int(servo_id)] = int(state.position[0])
        try:
            positions = pulses_to_positions(
                pulses,
                self._servo_ids,
                center_pulse=self._center_pulse,
                radians_per_tick=self._radians_per_tick,
                inverted_joints=self._inverted_joints,
            )
            link4 = forward_kinematics(self._chain, positions)
            camera = link4 @ self._hand_eye
        except (KeyError, ValueError) as error:
            self._fail('wrist FK failed: {}'.format(error))
            return
        if not np.all(np.isfinite(camera)):
            self._fail('wrist FK produced a non-finite pose')
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._camera_frame
        transform.transform.translation.x = float(camera[0, 3])
        transform.transform.translation.y = float(camera[1, 3])
        transform.transform.translation.z = float(camera[2, 3])
        quaternion = quaternion_from_matrix(camera[:3, :3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._broadcaster.sendTransform(transform)

        now = time.monotonic()
        self._last_success = now
        self._reads += 1
        if now - self._last_log >= 1.0:
            self._last_log = now
            angles = ' '.join(
                '{}={:+.3f}'.format(name, positions[name])
                for name in self._servo_ids)
            self.get_logger().info(
                'wrist_tf reads={} failures={} q=[{}] '
                'camera=({:+.3f}, {:+.3f}, {:+.3f}) '
                'view_z=({:+.3f}, {:+.3f}, {:+.3f})'.format(
                    self._reads, self._failures, angles,
                    camera[0, 3], camera[1, 3], camera[2, 3],
                    camera[0, 2], camera[1, 2], camera[2, 2]))

    def _fail(self, message):
        """Count a failure and log it at most once every two seconds."""
        self._failures += 1
        now = time.monotonic()
        if now - self._last_warn >= 2.0:
            self._last_warn = now
            self.get_logger().warn(message)


def main(args=None):
    """Run the measured wrist TF publisher."""
    rclpy.init(args=args)
    try:
        node = WristTfNode()
    except Exception as error:  # noqa: BLE001 - fatal startup error
        get_rclpy_logger('wrist_tf').fatal(
            'wrist_tf failed to start: {}'.format(error))
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch sends SIGINT again while the service client and TF
        # listener are torn down; ignore further interrupts so shutdown stays
        # clean (same guard as the edge node).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
