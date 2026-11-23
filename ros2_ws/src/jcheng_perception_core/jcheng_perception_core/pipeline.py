"""
The edge-side perception pipeline.

Data flow for one detection round:

    Pi 5: RGB capture ── compress ──> ROS 2 ──> laptop YOLO
    laptop: detections (capture stamp preserved) ──> ROS 2 ──> Pi 5
    Pi 5: nearest depth by capture stamp
          interpolated base<-camera TF by capture stamp
          back-project -> associate -> distance-weighted fusion -> Kalman
          publish FusedTarget at a fixed rate

No YOLO runs here. The only per-detection work is depth sampling, a 3x3
transform and a small Kalman update, so the motion/control path is never
waiting on inference.
"""

import dataclasses
from typing import Dict, List, Optional

import numpy as np

from .backproject import build_observation
from .config import CameraSpec, FusionConfig
from .time_buffer import FrameRing, LatencyWindow, PoseRingBuffer
from .tracking import TrackManager
from .types import (
    CameraHealth,
    Detection,
    DepthFrame,
    FrameMeta,
    Phase,
    PipelineStatus,
    TargetObservation,
    TrackStatus,
)
from .weighting import phase_for_observations


@dataclasses.dataclass
class PendingDetection:
    """A detection batch waiting for the short two-view coalescing window."""

    camera_id: str
    seq: int
    capture_ns: int
    received_ns: int
    deadline_ns: int
    detections: List[Detection]
    model_id: str = ''


@dataclasses.dataclass
class SyncResult:
    """The time-matched inputs for one detection batch."""

    frame: FrameMeta
    depth: DepthFrame
    pose: np.ndarray
    depth_age_ns: int
    pose_age_ns: int


class PerceptionPipeline:
    """Time-matched, two-view fusion pipeline (pure Python)."""

    def __init__(self, specs, fusion, static_fallbacks=None):
        """Create empty rings and a track manager for the given cameras."""
        self.specs: Dict[str, CameraSpec] = dict(specs)
        self.fusion: FusionConfig = fusion
        self.static_fallbacks: Dict[str, np.ndarray] = dict(
            static_fallbacks or {})
        self._rings = {}
        for camera_id, spec in self.specs.items():
            self._rings[camera_id] = {
                'frames': FrameRing(
                    max_items=spec.max_buffer_items_config,
                    max_age_ns=int(spec.frame_buffer_seconds * 1e9),
                ),
                'poses': PoseRingBuffer(
                    max_items=max(64, int(
                        spec.pose_buffer_seconds * 200)),
                    max_age_ns=int(spec.pose_buffer_seconds * 1e9),
                ),
            }
        self.tracks = TrackManager(self.specs, self.fusion)
        self.pending: List[PendingDetection] = []
        self.health: Dict[str, CameraHealth] = {
            camera_id: CameraHealth() for camera_id in self.specs
        }
        self._latencies = {
            camera_id: LatencyWindow() for camera_id in self.specs
        }
        self._last_prune_ns = 0
        self._phase_override: Optional[Phase] = None
        self._phase_override_until_ns = 0
        self.last_phase = Phase.SEARCH
        self._last_targets = []
        self.last_observations: List[TargetObservation] = []
        self._last_status = PipelineStatus()
        self.total_frames = 0
        self.total_detections = 0
        self.total_matched = 0
        self.total_dropped = 0
        self.max_pending = 64

    # ------------------------------------------------------------------ input
    def ingest_frame(self, camera_id, seq, capture_ns, received_ns, width,
                     height, frame_id, camera_matrix, distortion=None):
        """Store one RGB acquisition's metadata in the timestamp ring."""
        spec = self._spec(camera_id)
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        if camera_matrix.shape != (3, 3):
            raise ValueError('camera_matrix must be 3x3')
        distortion = np.zeros(5, dtype=np.float64) if distortion is None \
            else np.asarray(distortion, dtype=np.float64).reshape(-1)
        meta = FrameMeta(
            camera_id=camera_id,
            seq=int(seq),
            capture_ns=int(capture_ns),
            received_ns=int(received_ns),
            width=int(width),
            height=int(height),
            frame_id=str(frame_id),
            camera_matrix=camera_matrix,
            distortion=distortion,
        )
        self._rings[camera_id]['frames'].add_meta(capture_ns, meta)
        health = self.health[camera_id]
        health.frames_received += 1
        health.last_frame_ns = int(capture_ns)
        self.total_frames += 1
        self._maybe_prune(capture_ns)
        return meta

    def ingest_depth(self, camera_id, capture_ns, data, scale_m=0.001,
                     metric=False):
        """Store one depth frame keyed by its acquisition timestamp."""
        self._spec(camera_id)
        depth_frame = DepthFrame(
            data=np.asarray(data),
            scale_m=float(scale_m),
            metric=bool(metric),
            stamp_ns=int(capture_ns),
        )
        self._rings[camera_id]['frames'].add_depth(capture_ns, depth_frame)
        self.health[camera_id].depth_received += 1
        self._maybe_prune(capture_ns)
        return depth_frame

    def ingest_pose(self, camera_id, stamp_ns, transform):
        """Store one base<-camera TF sample in the pose ring."""
        self._spec(camera_id)
        matrix = np.asarray(transform, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError('transform must be 4x4')
        self._rings[camera_id]['poses'].add(stamp_ns, matrix)
        self.health[camera_id].tf_samples += 1

    def note_tf_failure(self, camera_id):
        """Count one failed TF sample."""
        self.health[camera_id].tf_failures += 1

    def note_uplink(self, camera_id):
        """Count one image published to the GPU."""
        self.health[camera_id].frames_uplinked += 1

    def note_rate_limited(self, camera_id):
        """Count one frame skipped by the uplink rate limiter."""
        self.health[camera_id].frames_rate_limited += 1

    def note_encode_failure(self, camera_id):
        """Count one JPEG encode failure."""
        self.health[camera_id].encode_failures += 1

    def ingest_detections(self, camera_id, seq, capture_ns, received_ns,
                          detections, model_id=''):
        """Queue one detection batch for time matching and fusion."""
        self._spec(camera_id)
        coalesce_ns = int(max(0.0, self.fusion.coalesce_ms) * 1e6)
        pending = PendingDetection(
            camera_id=camera_id,
            seq=int(seq),
            capture_ns=int(capture_ns),
            received_ns=int(received_ns),
            deadline_ns=int(received_ns) + coalesce_ns,
            detections=list(detections),
            model_id=str(model_id),
        )
        self.pending.append(pending)
        if len(self.pending) > self.max_pending:
            dropped = self.pending.pop(0)
            self.total_dropped += len(dropped.detections)
            self.health[dropped.camera_id].stale_detections += 1
        health = self.health[camera_id]
        health.detections_received += len(detections)
        health.last_detection_ns = int(capture_ns)
        latency_ms = max(0.0, (int(received_ns) - int(capture_ns)) / 1e6)
        health.last_latency_ms = latency_ms
        health.mean_latency_ms = self._latencies[camera_id].mean()
        self._latencies[camera_id].add(
            latency_ms, stamp_ns=int(received_ns))
        self.total_detections += len(detections)
        return pending

    # --------------------------------------------------------------- matching
    def _spec(self, camera_id):
        """Return a configured camera or raise a clear error."""
        try:
            return self.specs[str(camera_id)]
        except KeyError as error:
            raise KeyError(
                'camera {} is not configured'.format(camera_id)) from error

    def _maybe_prune(self, now_ns):
        """Prune all rings about once per second."""
        if int(now_ns) - self._last_prune_ns < 1_000_000_000:
            return
        self._last_prune_ns = int(now_ns)
        for ring in self._rings.values():
            ring['frames'].prune(now_ns)
            ring['poses'].prune(now_ns)

    def _pose_at(self, camera_id, capture_ns):
        """Return ``(pose, age_ns)`` for a capture time, or ``(None, 0)``."""
        spec = self.specs[camera_id]
        pose_ring = self._rings[camera_id]['poses']
        tolerance_ns = int(spec.pose_match_tolerance_ms * 1e6)
        pose = pose_ring.pose_at(
            capture_ns, max_gap_ns=max(2 * tolerance_ns, 20_000_000),
            max_delta_ns=tolerance_ns)
        if pose is not None:
            nearest = pose_ring.nearest(capture_ns, tolerance_ns)
            age = abs(nearest[0] - int(capture_ns)) if nearest else 0
            return pose, age
        nearest = pose_ring.nearest(capture_ns, tolerance_ns)
        if nearest is not None:
            return nearest[1].copy(), abs(nearest[0] - int(capture_ns))
        fallback = self.static_fallbacks.get(camera_id)
        if fallback is not None:
            return np.asarray(fallback, dtype=np.float64).copy(), 0
        return None, 0

    def synchronize(self, camera_id, capture_ns):
        """Return the frame, depth and pose matched to a capture timestamp."""
        spec = self.specs[camera_id]
        ring = self._rings[camera_id]
        frame_match = ring['frames'].meta.nearest(
            capture_ns, max_delta_ns=int(
                spec.frame_match_tolerance_ms * 1e6))
        if frame_match is None:
            self.health[camera_id].frame_match_failures += 1
            return None
        depth_match = ring['frames'].depth.nearest(
            capture_ns, max_delta_ns=int(
                spec.depth_match_tolerance_ms * 1e6))
        if depth_match is None:
            self.health[camera_id].depth_match_failures += 1
            return None
        depth_stamp, depth = depth_match
        # The depth pixel defines the 3-D point, so the camera pose must be
        # the pose at the depth frame's own stamp, not at the RGB stamp. For a
        # moving wrist camera this removes the depth/RGB rate mismatch from
        # the back-projection.
        pose, pose_age_ns = self._pose_at(camera_id, depth_stamp)
        if pose is None:
            self.health[camera_id].tf_match_failures += 1
            return None
        frame = frame_match[1]
        if depth.height != frame.height or depth.width != frame.width:
            self.health[camera_id].depth_match_failures += 1
            return None
        return SyncResult(
            frame=frame,
            depth=depth,
            pose=pose,
            depth_age_ns=abs(int(depth_stamp) - int(capture_ns)),
            pose_age_ns=int(pose_age_ns),
        )

    # ---------------------------------------------------------------- fusion
    def _build_observations(self, pending):
        """Convert pending batches into 3-D observations."""
        observations = []
        for item in pending:
            spec = self.specs[item.camera_id]
            sync = self.synchronize(item.camera_id, item.capture_ns)
            if sync is None:
                self.total_dropped += len(item.detections)
                continue
            health = self.health[item.camera_id]
            health.last_depth_age_ms = sync.depth_age_ns / 1e6
            health.last_frame_age_ms = (
                abs(sync.frame.capture_ns - item.capture_ns) / 1e6)
            for detection in item.detections:
                observation = build_observation(
                    detection, sync.frame, sync.depth, sync.pose,
                    spec, self.fusion)
                if observation is None:
                    self.total_dropped += 1
                    continue
                observation.capture_ns = int(sync.depth.stamp_ns)
                observations.append(observation)
                health.detections_matched += 1
                self.total_matched += 1
        return observations

    def _active_phase(self, now_ns):
        """Return the active phase override, if any."""
        if self._phase_override is not None and \
                int(now_ns) < self._phase_override_until_ns:
            return self._phase_override
        self._phase_override = None
        return None

    def flush(self, now_ns):
        """
        Process due detection batches and return the current target list.

        The function is safe to call at a fixed publication rate even when no
        detection is due; it then predicts existing tracks to ``now_ns``.
        """
        now_ns = int(now_ns)
        due = [item for item in self.pending if item.deadline_ns <= now_ns]
        if due:
            due_ids = {id(item) for item in due}
            self.pending = [
                item for item in self.pending if id(item) not in due_ids]
            due.sort(key=lambda item: item.capture_ns)
            observations = self._build_observations(due)
            self.last_observations = list(observations)
            override = self._active_phase(now_ns)
            if observations:
                phase = override or phase_for_observations(
                    observations, list(self.tracks.tracks.values()),
                    self.fusion)
            elif self.tracks.tracks:
                phase = override or self.last_phase
            else:
                phase = override or Phase.SEARCH
            if observations:
                assignments = self.tracks.associate(observations, now_ns)
                self.tracks.update(assignments, phase, now_ns)
                self.tracks.merge_duplicates(now_ns)
            self.last_phase = phase
        else:
            phase = self._active_phase(now_ns) or self.last_phase
            self.last_phase = phase
            self.tracks.merge_duplicates(now_ns)
        self.tracks.expire(now_ns)
        self._last_targets = self.tracks.targets(
            now_ns, phase,
            recheck_required=phase is Phase.DROP_RECHECK)
        return self._last_targets

    def targets(self, now_ns):
        """Return the last fused targets, predicted to ``now_ns``."""
        return self.tracks.targets(
            now_ns, self.last_phase,
            recheck_required=self.last_phase is Phase.DROP_RECHECK)

    # -------------------------------------------------------------- controls
    def set_phase_override(self, phase, now_ns, duration_s=None):
        """Force a phase (for example DROP_RECHECK) for a bounded time."""
        phase = Phase(phase)
        duration = self.fusion.recheck_duration_s if duration_s is None \
            else float(duration_s)
        self._phase_override = phase
        self._phase_override_until_ns = int(now_ns) + int(duration * 1e9)
        self.last_phase = phase

    def clear_phase_override(self):
        """Remove a forced phase."""
        self._phase_override = None
        self._phase_override_until_ns = 0

    def reset(self):
        """Drop tracks and pending detections, keeping the ring buffers."""
        self.tracks.reset()
        self.pending.clear()
        self.last_phase = Phase.SEARCH
        self._phase_override = None
        self._last_targets = []
        self.last_observations = []

    # --------------------------------------------------------------- status
    def status(self, now_ns):
        """Return an aggregated status snapshot for diagnostics."""
        now_ns = int(now_ns)
        confirmed = sum(
            1 for track in self.tracks.tracks.values()
            if track.status is TrackStatus.CONFIRMED)
        gpu_alive = any(
            health.last_detection_ns > 0
            and now_ns - health.last_detection_ns
            <= int(self.fusion.stale_gpu_s * 1e9)
            for health in self.health.values()
        )
        notes = []
        for camera_id, health in self.health.items():
            if health.tf_match_failures:
                notes.append('{}:tf_failures={}'.format(
                    camera_id, health.tf_match_failures))
            if health.depth_match_failures:
                notes.append('{}:depth_failures={}'.format(
                    camera_id, health.depth_match_failures))
        status = PipelineStatus(
            phase=self.last_phase,
            active_tracks=len(self.tracks.tracks),
            confirmed_tracks=confirmed,
            gpu_alive=gpu_alive,
            last_detection_ns=max(
                (health.last_detection_ns
                 for health in self.health.values()), default=0),
            notes=notes,
        )
        self._last_status = status
        return status

    @property
    def last_targets(self):
        """Return the targets from the most recent flush."""
        return list(self._last_targets)

    def take_observations(self):
        """Return and clear the observations produced by the last flush."""
        values = list(self.last_observations)
        self.last_observations = []
        return values
