"""
Target association and track lifecycle.

Detections arrive as independent 2-D boxes from two cameras. Association turns
them into persistent 3-D targets in the target frame, and the Kalman state
provides a prediction for the next association and for the planner between
detections. The policy is deliberately conservative: no label match, no update;
no measurement inside the gate, no merge.
"""

import dataclasses
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import FusionConfig
from .kalman import ConstantVelocityKalman
from .types import (
    FusedTarget,
    Phase,
    Role,
    TargetObservation,
    TrackStatus,
)
from .weighting import FusedMeasurement, fuse_observations


@dataclasses.dataclass
class Track:
    """One persistent target track."""

    target_id: str
    label: str
    class_id: int
    kf: ConstantVelocityKalman
    created_ns: int
    last_update_ns: int
    last_state_ns: int
    status: TrackStatus = TrackStatus.TENTATIVE
    hits: int = 1
    wrist_hits: int = 0
    external_hits: int = 0
    last_fused: Optional[FusedMeasurement] = None
    last_orientation: Optional[np.ndarray] = None
    last_orientation_sigma: float = 0.0
    last_status_ns: int = 0

    def age_s(self, now_ns):
        """Return the time since the last successful update."""
        return max(0.0, (int(now_ns) - int(self.last_update_ns)) / 1e9)

    def age_total_s(self, now_ns):
        """Return the time since the track was created."""
        return max(0.0, (int(now_ns) - int(self.created_ns)) / 1e9)


class TrackManager:
    """Associate observations to tracks and maintain their filters."""

    def __init__(self, specs, fusion):
        """Create an empty manager for the configured cameras."""
        self.specs = specs
        self.fusion = fusion
        self.tracks: Dict[str, Track] = {}
        self._counter = 0
        self._last_observations: List[TargetObservation] = []

    def reset(self):
        """Drop all tracks and restart identifiers at one."""
        self.tracks.clear()
        self._counter = 0
        self._last_observations = []

    def _new_track_id(self, label):
        """Return a stable, readable identifier for a new track."""
        self._counter += 1
        stem = ''.join(
            character if character.isalnum() else '_'
            for character in str(label)
        ).strip('_') or 'target'
        return '{}-{:03d}'.format(stem, self._counter)

    def _predicted_position(self, track, stamp_ns):
        """Return the constant-velocity prediction at ``stamp_ns``."""
        dt = max(0.0, (int(stamp_ns) - int(track.last_state_ns)) / 1e9)
        dt = min(dt, 0.5)
        return track.kf.position + track.kf.velocity * dt

    def _gate(self, track, stamp_ns):
        """Return an association gate in metres for this track."""
        age = max(0.0, (int(stamp_ns) - int(track.last_update_ns)) / 1e9)
        growth = self.fusion.association_gate_growth_m * (
            1.0 - math.exp(-age / 0.35))
        return (self.fusion.association_gate_m + growth
                + self.fusion.association_gate_per_second * age)

    def _labels_match(self, track, observation):
        """Return True when a track may accept this observation label."""
        if self.fusion.association_label_agnostic:
            return True
        return str(track.label) == str(observation.label)

    def associate(self, observations, now_ns):
        """
        Assign observations to tracks, creating tracks for unmatched ones.

        Returns a list of ``(track, observation)`` pairs in input order. A
        track may receive at most one observation per camera in one call, so two
        boxes from the same camera cannot collapse onto one track.
        """
        assignments: List[Tuple[Track, TargetObservation]] = []
        used_cameras: Dict[str, set] = {}
        for observation in observations:
            best_track = None
            best_distance = float('inf')
            for track in self.tracks.values():
                cameras = used_cameras.setdefault(track.target_id, set())
                if observation.camera_id in cameras:
                    continue
                if not self._labels_match(track, observation):
                    continue
                predicted = self._predicted_position(
                    track, observation.capture_ns)
                distance = float(np.linalg.norm(
                    np.asarray(observation.position_base) - predicted))
                gate = self._gate(track, observation.capture_ns)
                if distance <= gate and distance < best_distance:
                    best_track = track
                    best_distance = distance
            if best_track is not None:
                used_cameras.setdefault(
                    best_track.target_id, set()).add(observation.camera_id)
                assignments.append((best_track, observation))
                continue

            track = self._create_track(observation)
            used_cameras.setdefault(track.target_id, set()).add(
                observation.camera_id)
            assignments.append((track, observation))
        self._last_observations = list(observations)
        return assignments

    def _create_track(self, observation):
        """Create a tentative track from an unmatched observation."""
        target_id = self._new_track_id(observation.label)
        kf = ConstantVelocityKalman(
            observation.position_base,
            observation.covariance_base,
            accel_noise=self.fusion.kalman_accel_noise,
            measurement_floor_m=self.fusion.measurement_floor_m,
        )
        track = Track(
            target_id=target_id,
            label=str(observation.label),
            class_id=int(observation.class_id),
            kf=kf,
            created_ns=int(observation.capture_ns),
            last_update_ns=int(observation.received_ns),
            last_state_ns=int(observation.capture_ns),
            status=TrackStatus.TENTATIVE,
            hits=1,
            wrist_hits=1 if observation.role is Role.WRIST else 0,
            external_hits=1 if observation.role is Role.EXTERNAL else 0,
        )
        self.tracks[target_id] = track
        return track

    def apply_group(self, track, observations, phase, now_ns):
        """Fuse a track's fresh observations and update its filter."""
        fused = fuse_observations(
            observations, phase, self.specs, self.fusion)
        if fused is None:
            return None
        for observation in observations:
            observation.view_weight = float(
                fused.weights.get(observation.camera_id, 0.0))
        dt = max(0.0, (int(fused.stamp_ns) - int(track.last_state_ns)) / 1e9)
        dt = min(dt, 0.5)
        track.kf.predict(dt)
        track.kf.update(fused.position_base, fused.covariance_base)
        track.last_state_ns = int(fused.stamp_ns)
        track.last_update_ns = int(now_ns)
        track.hits += 1
        if Role.WRIST in [
                self.specs[camera_id].role
                for camera_id in fused.sources
                if camera_id in self.specs]:
            track.wrist_hits += 1
        if Role.EXTERNAL in [
                self.specs[camera_id].role
                for camera_id in fused.sources
                if camera_id in self.specs]:
            track.external_hits += 1
        track.last_fused = fused
        if fused.orientation_base is not None:
            track.last_orientation = np.asarray(
                fused.orientation_base, dtype=np.float64).copy()
            track.last_orientation_sigma = float(
                fused.orientation_sigma_rad)
        if track.hits >= self.fusion.confirm_hits:
            track.status = TrackStatus.CONFIRMED
        return fused

    def update(self, assignments, phase, now_ns):
        """Apply grouped assignments and return the fused measurements."""
        grouped: Dict[str, List[TargetObservation]] = {}
        for track, observation in assignments:
            grouped.setdefault(track.target_id, []).append(observation)
        fused_by_track = {}
        for target_id, observations in grouped.items():
            track = self.tracks.get(target_id)
            if track is None:
                continue
            fused = self.apply_group(track, observations, phase, now_ns)
            if fused is not None:
                fused_by_track[target_id] = fused
        return fused_by_track

    def _camera_set(self, track):
        """Return the cameras that contributed to a track's last fusion."""
        if track.last_fused is None:
            return set()
        return set(track.last_fused.sources)

    def merge_duplicates(self, now_ns):
        """Merge per-camera duplicate tracks that describe the same target.

        Two cameras can create one track each during startup or after a
        transient association miss. Because each view is always nearest to its
        own track, plain association never collapses them. This pass merges
        tracks with the same label, disjoint contributing cameras and centres
        within ``max(merge_gate_m, merge_sigma_scale * sigma)``. Two tracks
        that share a camera are never merged: they may be two real objects.
        """
        if not self.fusion.enable_track_merging:
            return []
        merged = []
        identifiers = list(self.tracks)
        for index, first_id in enumerate(identifiers):
            first = self.tracks.get(first_id)
            if first is None:
                continue
            for second_id in identifiers[index + 1:]:
                second = self.tracks.get(second_id)
                if second is None:
                    continue
                if not self.fusion.association_label_agnostic and \
                        first.label != second.label:
                    continue
                first_cameras = self._camera_set(first)
                second_cameras = self._camera_set(second)
                if not first_cameras or not second_cameras:
                    continue
                first_position = self._predicted_position(first, now_ns)
                second_position = self._predicted_position(second, now_ns)
                distance = float(np.linalg.norm(
                    first_position - second_position))
                overlap = bool(first_cameras & second_cameras)
                if overlap:
                    # Same-camera duplicates can occur after a transient missed
                    # association. Two objects closer than this are below the
                    # depth/pixel resolution anyway; keep a strict gate so two
                    # clearly separated same-camera objects stay distinct.
                    gate = self.fusion.merge_gate_overlap_m
                else:
                    sigma = math.sqrt(
                        max(0.0, float(np.trace(
                            first.kf.position_covariance)) / 3.0)
                        + max(0.0, float(np.trace(
                            second.kf.position_covariance)) / 3.0))
                    gate = max(
                        self.fusion.merge_gate_m,
                        self.fusion.merge_sigma_scale * sigma,
                    )
                if distance > gate:
                    continue
                primary, secondary = self._primary_track(first, second)
                self._absorb(primary, secondary)
                self.tracks.pop(secondary.target_id, None)
                merged.append((
                    primary.target_id, secondary.target_id, distance))
                if first is secondary:
                    break
        return merged

    @staticmethod
    def _primary_track(first, second):
        """Choose the track that keeps its identity when merging."""
        if first.hits != second.hits:
            return (first, second) if first.hits > second.hits \
                else (second, first)
        if first.created_ns != second.created_ns:
            return (first, second) if first.created_ns < second.created_ns \
                else (second, first)
        return (first, second)

    def _absorb(self, primary, secondary):
        """Fold the secondary track's state into the primary track."""
        if secondary.last_fused is not None and \
                secondary.last_state_ns > primary.last_state_ns:
            dt = min(
                0.5,
                max(0.0, (secondary.last_state_ns
                          - primary.last_state_ns) / 1e9))
            primary.kf.predict(dt)
            primary.kf.update(
                secondary.last_fused.position_base,
                secondary.last_fused.covariance_base)
            primary.last_state_ns = secondary.last_state_ns
        primary.hits = max(primary.hits, secondary.hits)
        primary.wrist_hits = max(
            primary.wrist_hits, secondary.wrist_hits)
        primary.external_hits = max(
            primary.external_hits, secondary.external_hits)
        primary.created_ns = min(primary.created_ns, secondary.created_ns)
        primary.last_update_ns = max(
            primary.last_update_ns, secondary.last_update_ns)
        if primary.last_orientation is None and \
                secondary.last_orientation is not None:
            primary.last_orientation = secondary.last_orientation.copy()
            primary.last_orientation_sigma = \
                secondary.last_orientation_sigma

    def expire(self, now_ns):
        """Update lifecycle states and remove tracks that are too old."""
        remove = []
        for target_id, track in self.tracks.items():
            age = track.age_s(now_ns)
            if track.status is TrackStatus.TENTATIVE and \
                    age > self.fusion.tentative_timeout_s:
                remove.append(target_id)
            elif age > self.fusion.lost_after_s + 0.5:
                remove.append(target_id)
            elif age > self.fusion.lost_after_s:
                track.status = TrackStatus.LOST
            elif age > self.fusion.coast_after_s:
                track.status = TrackStatus.COASTING
            elif track.hits >= self.fusion.confirm_hits:
                track.status = TrackStatus.CONFIRMED
        for target_id in remove:
            self.tracks.pop(target_id, None)

    def phase(self, phase_override=None):
        """Return the task phase from tracks and the latest observations."""
        if phase_override is not None:
            return phase_override
        from .weighting import phase_for_observations

        return phase_for_observations(
            self._last_observations,
            list(self.tracks.values()),
            self.fusion,
        )

    def targets(self, now_ns, phase, publish_tentative=False,
                recheck_required=False):
        """Return planners-ready targets predicted to ``now_ns``."""
        targets = []
        for track in self.tracks.values():
            if track.status is TrackStatus.LOST:
                continue
            if track.status is not TrackStatus.CONFIRMED and \
                    not publish_tentative:
                continue
            if track.status is TrackStatus.TENTATIVE and \
                    track.hits < 1:
                continue
            dt = max(0.0, (int(now_ns) - int(track.last_state_ns)) / 1e9)
            dt = min(dt, 1.0)
            position = track.kf.position + track.kf.velocity * dt
            covariance = track.kf.covariance[:3, :3].copy()
            covariance = covariance + np.eye(3) * (
                self.fusion.measurement_floor_m ** 2
                + self.fusion.kalman_accel_noise * dt ** 3 / 3.0)
            covariance = 0.5 * (covariance + covariance.T)
            if float(np.max(np.diag(covariance))) > \
                    self.fusion.max_position_variance_m2:
                continue

            age = track.age_s(now_ns)
            status = track.status
            if age > self.fusion.coast_after_s:
                status = TrackStatus.COASTING
            decay = math.exp(-max(0.0, age - 0.25) / 0.8)
            confidence = 0.0
            weights = {}
            depths = {}
            sources = []
            occluded = False
            orientation = track.last_orientation
            orientation_sigma = track.last_orientation_sigma
            if track.last_fused is not None:
                confidence = float(track.last_fused.confidence) * decay
                weights = dict(track.last_fused.weights)
                depths = dict(track.last_fused.depths)
                sources = list(track.last_fused.sources)
                occluded = track.last_fused.occlusion_ratio >= 0.5
                if orientation is None:
                    orientation = track.last_fused.orientation_base
            covariance6 = np.zeros((6, 6), dtype=np.float64)
            covariance6[:3, :3] = covariance
            covariance6[3, 3] = 1.0
            covariance6[4, 4] = 1.0
            if orientation is not None:
                covariance6[5, 5] = max(
                    1e-4, float(orientation_sigma) ** 2)
            else:
                covariance6[5, 5] = 1.0
            targets.append(FusedTarget(
                target_id=track.target_id,
                label=track.label,
                class_id=track.class_id,
                stamp_ns=int(now_ns),
                position_base=position,
                orientation_base=orientation,
                covariance_6x6=covariance6,
                confidence=confidence,
                phase=phase,
                status=status,
                weights=weights,
                depth_by_camera=depths,
                occluded=occluded,
                recheck_required=bool(
                    recheck_required
                    and phase is Phase.DROP_RECHECK
                    and Role.EXTERNAL in [
                        self.specs[camera_id].role
                        for camera_id in sources if camera_id in self.specs
                    ]),
                source_cameras=sources,
                age_s=track.age_total_s(now_ns),
            ))
        targets.sort(key=lambda target: target.confidence, reverse=True)
        return targets

    @property
    def observation_cache(self):
        """Return the observations used by the last association call."""
        return list(self._last_observations)
