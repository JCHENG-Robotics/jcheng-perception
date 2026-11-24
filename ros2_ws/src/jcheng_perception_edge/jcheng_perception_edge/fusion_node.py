"""
Fusion-only edge process for the split (per-camera ingress) deployment.

Each camera runs its own ``perception_edge`` ingress process with
``camera_filter:=<camera>``; the ingress owns the camera subscriptions, depth/TF
matching, back-projection, uplink and publishes ``TargetObservation`` messages
on ``/perception/observations/<camera>``. This node subscribes to those small
messages, runs association/Kalman/fusion, and publishes the planner-facing
targets and status.

Separating the large camera readers from the fusion participant means a wedged
Aurora depth (or D435i colour) reader only stops that ingress; a launch respawn
can restart it without touching the other camera or the fusion state.
"""

import collections
import dataclasses
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovariance
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from jcheng_perception_core import (
    Phase,
    TargetObservation,
    TrackStatus,
    TrackManager,
    load_camera_specs,
    load_fusion_config,
    phase_for_observations,
)
from jcheng_perception_msgs.msg import (
    FusedTarget,
    FusedTargetArray,
    PerceptionStatus,
    TargetObservation as TargetObservationMsg,
)


def _state_qos(depth=10):
    """Best-effort QoS for periodic state."""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _header_stamp_ns(header, fallback_ns):
    """Return a header stamp in nanoseconds."""
    stamp = Time.from_msg(header.stamp).nanoseconds
    return stamp if stamp > 0 else int(fallback_ns)


def _covariance6x6(covariance3, yaw_sigma, has_orientation):
    """Build the 6x6 pose covariance with unobserved roll/pitch marked."""
    covariance = np.zeros(36, dtype=np.float64)
    covariance[:9] = np.asarray(covariance3, dtype=np.float64).reshape(9)
    covariance[21] = 1.0
    covariance[28] = 1.0
    covariance[35] = max(1e-4, float(yaw_sigma) ** 2) \
        if has_orientation else 1.0
    return covariance.tolist()


@dataclasses.dataclass
class PendingObservation:
    """One incoming observation waiting for the coalescing window."""

    deadline_ns: int
    observation: TargetObservation


class PerceptionFusionNode(Node):
    """Associate, filter and publish targets from ingress observations."""

    def __init__(self):
        """Load config, subscribe to ingress observations, create timers."""
        super().__init__('jcheng_perception_fusion')
        self.declare_parameter(
            'cameras_config', '')
        self.declare_parameter(
            'fusion_config', '')
        self.declare_parameter('target_frame', '')
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('status_rate_hz', 0.0)

        cameras_path = str(self.get_parameter('cameras_config').value)
        fusion_path = str(self.get_parameter('fusion_config').value)
        if not cameras_path or not fusion_path:
            raise RuntimeError(
                'cameras_config and fusion_config are required')
        self.specs = load_camera_specs(cameras_path)
        self.fusion = load_fusion_config(fusion_path)
        target_frame = str(self.get_parameter('target_frame').value).strip()
        if target_frame:
            self.fusion.target_frame = target_frame
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate > 0.0:
            self.fusion.publish_rate_hz = publish_rate
        status_rate = float(self.get_parameter('status_rate_hz').value)
        if status_rate > 0.0:
            self.fusion.status_rate_hz = status_rate

        self.tracks = TrackManager(self.specs, self.fusion)
        self.pending = []
        self.max_pending = 256
        self.last_phase = Phase.SEARCH
        self.total_observations = 0
        self._last_status_ns = 0
        self._observation_times = {
            camera_id: collections.deque(maxlen=90)
            for camera_id in self.specs
        }
        self._latencies = {
            camera_id: collections.deque(maxlen=90)
            for camera_id in self.specs
        }
        self._logged_bad_observations = set()
        self._logged_bad_targets = set()
        self.target_publisher = self.create_publisher(
            FusedTargetArray, '/perception/targets', _state_qos())
        self.status_publisher = self.create_publisher(
            PerceptionStatus, '/perception/status', _state_qos(5))
        for camera_id in self.specs:
            topic = '/perception/observations/{}'.format(camera_id)
            self.create_subscription(
                TargetObservationMsg, topic,
                lambda message, camera=camera_id: self.on_observation(
                    camera, message),
                _state_qos())
        self.create_timer(
            1.0 / max(1.0, float(self.fusion.publish_rate_hz)),
            self.on_publish_timer)
        self.get_logger().info(
            'fusion ready: cameras={} target_frame={}'.format(
                ','.join(self.specs), self.fusion.target_frame))

    def _now_ns(self):
        """Return the current ROS clock in nanoseconds."""
        return self.get_clock().now().nanoseconds

    def _observation_from_message(self, camera_id, message, now_ns):
        """Convert a TargetObservation message into the core datatype."""
        pose = message.pose.pose
        covariance = np.asarray(
            message.pose.covariance, dtype=np.float64).reshape(6, 6)
        orientation = np.array([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ], dtype=np.float64)
        if not np.allclose(orientation, [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            orientation_base = orientation
            orientation_sigma = float(np.sqrt(max(1e-6, covariance[5, 5])))
        else:
            orientation_base = None
            orientation_sigma = 0.0
        position = np.array([
            pose.position.x, pose.position.y, pose.position.z])
        if not np.all(np.isfinite(position)) or \
                float(np.linalg.norm(position)) > 2.0:
            key = (camera_id, float(position[0]), float(position[1]))
            if key not in self._logged_bad_observations:
                self._logged_bad_observations.add(key)
                self.get_logger().warn(
                    '{}: rejecting bad observation position {}'.format(
                        camera_id, position))
            return None
        spec = self.specs[camera_id]
        return TargetObservation(
            camera_id=camera_id,
            role=spec.role,
            label=str(message.label),
            class_id=int(message.class_id),
            capture_ns=int(_header_stamp_ns(message.header, now_ns)),
            received_ns=int(now_ns),
            frame_seq=int(message.frame_seq),
            frame_id_camera=str(message.camera_frame),
            detection_score=float(message.detection_score),
            keypoint_score=float(message.keypoint_score),
            position_base=position,
            covariance_base=covariance[:3, :3],
            depth_m=float(message.depth_m),
            valid_depth_fraction=float(message.valid_depth_fraction),
            occlusion_ratio=float(message.occlusion_ratio),
            orientation_base=orientation_base,
            orientation_sigma_rad=orientation_sigma,
            view_weight=float(message.view_weight),
        )

    def on_observation(self, camera_id, message):
        """Queue one ingress observation."""
        now_ns = self._now_ns()
        observation = self._observation_from_message(
            camera_id, message, now_ns)
        if observation is None:
            return
        deadline = now_ns + int(max(0.0, self.fusion.coalesce_ms) * 1e6)
        self.pending.append(PendingObservation(deadline, observation))
        if len(self.pending) > self.max_pending:
            self.pending.pop(0)
        self.total_observations += 1
        self._observation_times[camera_id].append(now_ns)
        self._latencies[camera_id].append(max(
            0.0, (now_ns - observation.capture_ns) / 1e6))

    def _fps(self, camera_id):
        """Return the ingress observation rate for one camera."""
        stamps = self._observation_times[camera_id]
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) * 1e9 / float(stamps[-1] - stamps[0])

    def _flush(self, now_ns):
        """Apply due observations and return the current targets."""
        due = [item for item in self.pending if item.deadline_ns <= now_ns]
        if due:
            self.pending = [
                item for item in self.pending
                if item.deadline_ns > now_ns]
            due.sort(key=lambda item: item.observation.capture_ns)
            observations = [item.observation for item in due]
            self.last_phase = phase_for_observations(
                observations, list(self.tracks.tracks.values()),
                self.fusion)
            assignments = self.tracks.associate(observations, now_ns)
            self.tracks.update(assignments, self.last_phase, now_ns)
            self.tracks.merge_duplicates(now_ns)
        self.tracks.expire(now_ns)
        return self.tracks.targets(
            now_ns, self.last_phase,
            recheck_required=self.last_phase is Phase.DROP_RECHECK)

    def on_publish_timer(self):
        """Flush observations, publish targets and periodic status."""
        now_ns = self._now_ns()
        targets = self._flush(now_ns)
        self.publish_targets(targets, now_ns)
        if now_ns - self._last_status_ns >= int(
                1e9 / max(self.fusion.status_rate_hz, 1e-6)):
            self.publish_status(now_ns)
            self._last_status_ns = now_ns

    def publish_targets(self, targets, now_ns):
        """Publish the fused target array."""
        external_ids = [
            camera_id for camera_id, spec in self.specs.items()
            if spec.role.value == 'external']
        wrist_ids = [
            camera_id for camera_id, spec in self.specs.items()
            if spec.role.value == 'wrist']
        message = FusedTargetArray()
        message.header.stamp = Time(nanoseconds=int(now_ns)).to_msg()
        message.header.frame_id = self.fusion.target_frame
        for target in targets:
            position = np.asarray(target.position_base, dtype=np.float64)
            if not np.all(np.isfinite(position)) or \
                    float(np.linalg.norm(position)) > float(
                        self.fusion.max_target_distance_m):
                if target.target_id not in self._logged_bad_targets:
                    self._logged_bad_targets.add(target.target_id)
                    track = self.tracks.tracks.get(target.target_id)
                    detail = ''
                    if track is not None:
                        samples = {
                            camera: [int(s[0]),
                                     [round(float(v), 3) for v in s[1]]]
                            for camera, s in track.view_samples.items()
                        }
                        last = None
                        if track.last_fused is not None:
                            last = [round(float(v), 3) for v in
                                    track.last_fused.position_base]
                        detail = ' kf={} v={} off={} last={} samples={}'.format(
                            [round(float(v), 3) for v in track.kf.position],
                            [round(float(v), 3) for v in track.kf.velocity],
                            [round(float(v), 3) for v in track.view_offset],
                            last, samples)
                    self.get_logger().warn(
                        'bad target {} position {} cov={}{}'.format(
                            target.target_id, position,
                            np.round(np.diag(target.covariance_6x6[:3, :3]), 8),
                            detail))
                continue
            item = FusedTarget()
            item.header.stamp = Time(nanoseconds=int(now_ns)).to_msg()
            item.header.frame_id = self.fusion.target_frame
            item.target_id = target.target_id
            item.label = target.label
            item.class_id = int(target.class_id)
            item.pose = self._pose_with_covariance(
                target.position_base,
                target.orientation_base,
                target.covariance_6x6[:3, :3],
                float(np.sqrt(max(1e-6, target.covariance_6x6[5, 5]))),
                covariance6=target.covariance_6x6,
            )
            item.confidence = float(target.confidence)
            item.phase = int(target.phase)
            item.status = int(target.status)
            item.weight_external = float(sum(
                target.weights.get(camera_id, 0.0)
                for camera_id in external_ids))
            item.weight_wrist = float(sum(
                target.weights.get(camera_id, 0.0)
                for camera_id in wrist_ids))
            item.depth_external_m = float(max(
                [target.depth_by_camera.get(camera_id, 0.0)
                 for camera_id in external_ids] or [0.0]))
            item.depth_wrist_m = float(max(
                [target.depth_by_camera.get(camera_id, 0.0)
                 for camera_id in wrist_ids] or [0.0]))
            item.occluded = bool(target.occluded)
            item.recheck_required = bool(target.recheck_required)
            item.source_cameras = list(target.source_cameras)
            item.age_s = float(target.age_s)
            message.targets.append(item)
        self.target_publisher.publish(message)

    def publish_status(self, now_ns):
        """Publish a fusion status derived from ingress observations.

        The split deployment no longer sees raw camera counters here;
        ``total_frames``/``total_detections`` are observation counts and
        ``camera_fps`` is the per-camera observation rate. Ingress health
        (camera frames/depth) is in each ingress process log/status.
        """
        now_ns = int(now_ns)
        message = PerceptionStatus()
        message.header.stamp = Time(nanoseconds=now_ns).to_msg()
        message.header.frame_id = self.fusion.target_frame
        message.camera_ids = list(self.specs)
        message.camera_fps = [
            float(self._fps(camera_id)) for camera_id in self.specs]
        message.total_frames = int(self.total_observations)
        message.total_detections = int(self.total_observations)
        message.detections_matched = int(self.total_observations)
        message.detections_dropped = 0
        message.match_success_rate = 1.0
        latencies = [
            float(np.mean(values))
            for values in self._latencies.values() if values]
        message.mean_detection_latency_ms = float(
            np.mean(latencies) if latencies else 0.0)
        message.mean_depth_match_age_ms = 0.0
        message.phase = int(self.last_phase)
        last_observation_ns = max(
            (values[-1] for values in self._observation_times.values()
             if values), default=0)
        message.gpu_alive = bool(
            last_observation_ns > 0 and
            now_ns - last_observation_ns <= int(
                self.fusion.stale_gpu_s * 1e9))
        active = len(self.tracks.tracks)
        confirmed = sum(
            1 for track in self.tracks.tracks.values()
            if track.status is TrackStatus.CONFIRMED)
        message.active_tracks = int(min(255, active))
        message.confirmed_tracks = int(min(255, confirmed))
        notes = []
        for camera_id in self.specs:
            values = self._observation_times[camera_id]
            if values:
                notes.append('{}:obs_age_ms={:.0f}'.format(
                    camera_id, (now_ns - values[-1]) / 1e6))
            else:
                notes.append('{}:no_observations'.format(camera_id))
        message.notes = notes
        self.status_publisher.publish(message)

    def _pose_with_covariance(self, position, orientation, covariance3,
                              yaw_sigma, covariance6=None):
        """Build a PoseWithCovariance with a 6x6 row-major covariance."""
        message = PoseWithCovariance()
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        if orientation is None:
            message.pose.orientation.w = 1.0
        else:
            message.pose.orientation.x = float(orientation[0])
            message.pose.orientation.y = float(orientation[1])
            message.pose.orientation.z = float(orientation[2])
            message.pose.orientation.w = float(orientation[3])
        if covariance6 is None:
            message.covariance = _covariance6x6(
                covariance3, yaw_sigma, orientation is not None)
        else:
            message.covariance = [
                float(value) for value in np.asarray(
                    covariance6, dtype=np.float64).reshape(36)
            ]
        return message


def main(args=None):
    """Run the fusion node."""
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
