"""
Distance-aware two-view weighting.

Both cameras measure the same physical target, but their errors are not equal:
stereo depth error grows with range squared, the wrist camera has a narrow
working distance, and the two views have different roles in the task. The
weight of a view is therefore

    precision_i = role_prior(phase, role) * quality_i / sigma_i^2

where ``quality`` includes detector confidence, keypoint support, occlusion and
a range penalty, and ``sigma_i^2`` is the mean diagonal of the measurement
covariance. Weights are normalised across the views that actually observed the
target in the current fusion window. The fused covariance is the inverse of the
summed information, so one good view is never diluted by a stale or occluded
one.
"""

import dataclasses
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import CameraSpec, FusionConfig
from .types import Phase, Role, TargetObservation


@dataclasses.dataclass
class FusedMeasurement:
    """A weighted combination of observations from one or two views."""

    position_base: np.ndarray
    covariance_base: np.ndarray
    weights: Dict[str, float]
    factors: Dict[str, float]
    stamp_ns: int
    depths: Dict[str, float]
    sources: List[str]
    confidence: float
    occlusion_ratio: float
    orientation_base: Optional[np.ndarray]
    orientation_sigma_rad: float

    def __post_init__(self):
        """Normalise arrays."""
        self.position_base = np.asarray(
            self.position_base, dtype=np.float64).reshape(3)
        self.covariance_base = np.asarray(
            self.covariance_base, dtype=np.float64).reshape(3, 3)
        if self.orientation_base is not None:
            self.orientation_base = np.asarray(
                self.orientation_base, dtype=np.float64).reshape(4)


def range_penalty(depth_m, spec):
    """Return a 0..1 penalty for measurements outside a camera's range."""
    depth = float(depth_m)
    if depth < spec.min_range_m:
        excess = (spec.min_range_m - depth) / max(
            spec.range_penalty_width_m, 1e-6)
        return max(0.05, math.exp(-excess * excess))
    if depth > spec.max_range_m:
        excess = (depth - spec.max_range_m) / max(
            spec.range_penalty_width_m, 1e-6)
        return max(0.05, math.exp(-excess * excess))
    return 1.0


def view_factor(observation, phase, spec, fusion):
    """
    Return the unnormalised information factor for one observation.

    The factor is a relative precision multiplier. Role priors change which
    view dominates in a task phase; quality and range terms down-weight a
    weak, occluded or out-of-range detection.
    """
    prior = fusion.prior(phase, observation.role)
    if prior <= 0.0:
        return 0.0
    quality = max(float(observation.detection_score), 0.05)
    if observation.keypoint_score > 0.0:
        quality *= 1.0 + 0.5 * float(observation.keypoint_score)
    quality *= max(
        0.05, 1.0 - fusion.occlusion_penalty * observation.occlusion_ratio)
    quality *= range_penalty(observation.depth_m, spec)
    variance = max(observation.variance_trace, 1e-10)
    return float(prior * quality / variance)


def fuse_observations(observations, phase, specs, fusion):
    """
    Fuse the fresh observations of one target with dynamic weights.

    Returns ``None`` when no observation survives the minimum-weight filter.
    The published weights are normalised so the planner can log which view
    drove the estimate.
    """
    if not observations:
        return None
    entries = []
    for observation in observations:
        spec = specs.get(observation.camera_id)
        if spec is None:
            continue
        factor = view_factor(observation, phase, spec, fusion)
        if factor <= 0.0:
            continue
        entries.append((observation, factor))
    if not entries:
        return None
    max_factor = max(factor for _, factor in entries)
    if max_factor <= 0.0:
        return None
    entries = [
        (observation, factor / max_factor)
        for observation, factor in entries
    ]

    information = np.zeros((3, 3), dtype=np.float64)
    factors = {}
    for observation, factor in entries:
        covariance = np.asarray(
            observation.covariance_base, dtype=np.float64)
        covariance = 0.5 * (covariance + covariance.T)
        covariance = covariance + np.eye(3) * 1e-12
        try:
            precision = np.linalg.inv(covariance)
        except np.linalg.LinAlgError:
            precision = np.linalg.pinv(covariance)
        information += factor * precision
        factors[observation.camera_id] = float(factor)
    total_factor = sum(factors.values())
    weights = {
        camera_id: factor / total_factor
        for camera_id, factor in factors.items()
    }
    weights = {
        camera_id: weight for camera_id, weight in weights.items()
        if weight >= fusion.min_view_weight
    }
    if not weights:
        return None
    weight_sum = sum(weights.values())
    weights = {
        camera_id: weight / weight_sum
        for camera_id, weight in weights.items()
    }

    try:
        fused_covariance = np.linalg.inv(information)
    except np.linalg.LinAlgError:
        fused_covariance = np.linalg.pinv(information)
    if not np.all(np.isfinite(fused_covariance)):
        return None
    fused_covariance = 0.5 * (fused_covariance + fused_covariance.T)

    position = np.zeros(3, dtype=np.float64)
    confidence = 0.0
    occlusion = 0.0
    depths = {}
    for observation, _ in entries:
        weight = weights.get(observation.camera_id, 0.0)
        if weight <= 0.0:
            continue
        position += weight * observation.position_base
        confidence += weight * float(observation.detection_score)
        occlusion += weight * float(observation.occlusion_ratio)
        depths[observation.camera_id] = float(observation.depth_m)
    confidence *= max(0.25, 1.0 - 0.5 * occlusion)
    confidence = float(min(1.0, max(0.0, confidence)))

    orientation = None
    orientation_sigma = 0.0
    best_weight = -1.0
    for observation, _ in entries:
        weight = weights.get(observation.camera_id, 0.0)
        if weight > best_weight and observation.orientation_base is not None:
            best_weight = weight
            orientation = np.asarray(
                observation.orientation_base, dtype=np.float64).copy()
            orientation_sigma = float(observation.orientation_sigma_rad)

    sources = [
        observation.camera_id for observation, _ in entries
        if observation.camera_id in weights
    ]
    stamp_ns = max(
        int(observation.capture_ns)
        for observation, _ in entries
        if observation.camera_id in weights
    )
    return FusedMeasurement(
        position_base=position,
        covariance_base=fused_covariance,
        weights=weights,
        factors=factors,
        stamp_ns=stamp_ns,
        depths=depths,
        sources=sources,
        confidence=confidence,
        occlusion_ratio=float(min(1.0, occlusion)),
        orientation_base=orientation,
        orientation_sigma_rad=orientation_sigma,
    )


def phase_for_observations(observations, tracks, fusion):
    """
    Derive the task phase from the current observations and tracks.

    A wrist observation close to the arm and confirmed over several frames
    moves the task to FINE. DROP_RECHECK is an explicit operator/service
    decision, not an automatic consequence of geometry.
    """
    if not tracks and not observations:
        return Phase.SEARCH
    wrist_close = False
    wrist_seen = False
    external_seen = False
    for observation in observations:
        if observation.role is Role.WRIST:
            wrist_seen = True
            if observation.depth_m <= fusion.fine_distance_m:
                wrist_close = True
        elif observation.role is Role.EXTERNAL:
            external_seen = True
    if wrist_close:
        for track in tracks:
            if track.wrist_hits >= fusion.fine_wrist_hits:
                return Phase.FINE
        return Phase.ACQUIRE
    if wrist_seen and external_seen:
        return Phase.ACQUIRE
    if wrist_seen:
        return Phase.ACQUIRE
    if external_seen:
        return Phase.COARSE
    return Phase.SEARCH
