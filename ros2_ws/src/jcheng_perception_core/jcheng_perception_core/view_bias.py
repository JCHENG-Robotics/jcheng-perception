"""
Per-view systematic offset correction.

Each view measures a different physical point of a 3-D object: the external
camera's bounding-box centre lands on the side facing it, the wrist camera's on
the top. On top of that, each camera chain has its own extrinsic/depth bias.
The measurement model used here is

    z_i(t) = x(t) + b_i + m_i + n_i(t)

with ``b_i`` the camera-chain bias (constant, base frame), ``m_i`` the
measurement-point offset of the tracked object (constant for a rigid object
under slow motion) and ``n_i`` zero-mean noise. Only

    c_i = b_i + m_i

is observable from the two views: adding a constant to every ``c_i`` shifts the
absolute target and leaves all differences unchanged. The wrist view is
therefore the reference (``c_wrist = 0``) and only the external offset is
estimated. ``config/view_bias.yaml`` holds the chain-level part ``b_ext``; the
object-dependent ``m_ext`` is estimated online per track in
:class:`~jcheng_perception_core.tracking.TrackManager`.

Sign convention: ``offset_m`` is the correction **added** to the raw
back-projected position, so a config offset of ``[+0.0099, -0.0018, +0.0094]``
means "add 9.9 mm to x, subtract 1.8 mm from y, add 9.4 mm to z". ``sigma_m`` is
the 1σ uncertainty of that correction and is added in quadrature to the
measurement covariance, so a provisional correction cannot look more precise
than it is.
"""

import dataclasses
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from .config import load_yaml
from .types import TargetObservation


@dataclasses.dataclass
class ViewBias:
    """Constant correction for one camera."""

    camera_id: str
    offset_m: np.ndarray
    covariance_m2: np.ndarray

    def __post_init__(self):
        """Normalise the arrays."""
        self.offset_m = np.asarray(
            self.offset_m, dtype=np.float64).reshape(3)
        self.covariance_m2 = np.asarray(
            self.covariance_m2, dtype=np.float64).reshape(3, 3)


class ViewBiasModel:
    """Apply per-view offsets to observations and inflate their covariance."""

    def __init__(self, entries: Optional[Dict[str, ViewBias]] = None,
                 reference_camera_id: Optional[str] = None):
        """Create a model from parsed entries (default: no correction)."""
        self.entries: Dict[str, ViewBias] = dict(entries or {})
        self.reference_camera_id = reference_camera_id

    @classmethod
    def empty(cls):
        """Return a model that changes nothing."""
        return cls()

    @classmethod
    def from_config(cls, path):
        """Load ``view_bias.yaml``; a missing/empty file yields no correction."""
        document = load_yaml(path) if path else {}
        if not document:
            return cls.empty()
        reference = document.get('reference_camera')
        entries = {}
        views = document.get('views') or {}
        if not isinstance(views, dict):
            raise ValueError('{}: views must be a mapping'.format(path))
        for camera_id, entry in views.items():
            if not isinstance(entry, dict):
                raise ValueError(
                    '{}: view {} must be a mapping'.format(path, camera_id))
            offset = entry.get('offset_m', [0.0, 0.0, 0.0])
            sigma = entry.get('sigma_m', [0.0, 0.0, 0.0])
            offset = np.asarray(offset, dtype=np.float64).reshape(3)
            sigma = np.asarray(sigma, dtype=np.float64).reshape(3)
            if np.any(sigma < 0.0):
                raise ValueError(
                    '{}: view {} sigma_m must not be negative'.format(
                        path, camera_id))
            entries[str(camera_id)] = ViewBias(
                camera_id=str(camera_id),
                offset_m=offset,
                covariance_m2=np.diag(sigma * sigma),
            )
        return cls(entries, reference_camera_id=reference)

    def offset(self, camera_id):
        """Return the correction vector for a camera (zeros if unknown)."""
        entry = self.entries.get(str(camera_id))
        if entry is None:
            return np.zeros(3, dtype=np.float64)
        return entry.offset_m.copy()

    def covariance(self, camera_id):
        """Return the correction uncertainty matrix for a camera."""
        entry = self.entries.get(str(camera_id))
        if entry is None:
            return np.zeros((3, 3), dtype=np.float64)
        return entry.covariance_m2.copy()

    def correct(self, observation):
        """Return a copy of ``observation`` with the view bias removed."""
        entry = self.entries.get(str(observation.camera_id))
        if entry is None or (
                not np.any(entry.offset_m)
                and not np.any(entry.covariance_m2)):
            return observation
        corrected = dataclasses.replace(
            observation,
            position_base=(
                np.asarray(observation.position_base, dtype=np.float64)
                + entry.offset_m),
            covariance_base=(
                np.asarray(observation.covariance_base, dtype=np.float64)
                + entry.covariance_m2),
        )
        return corrected


def paired_offsets(observations: Iterable[TargetObservation],
                   reference_camera_id='aurora_wrist',
                   max_pair_age_s=1.0):
    """
    Return time-matched ``(camera_id, other - reference)`` residual samples.

    ``observations`` may contain any number of cameras and labels. The function
    pairs the nearest observation from each non-reference camera with each
    reference observation of the same label, subject to ``max_pair_age_s``.
    Used by the offline calibration/analysis scripts; the online per-track
    offset lives in ``TrackManager``.
    """
    by_label = {}
    for observation in observations:
        by_label.setdefault(str(observation.label), []).append(observation)
    residuals = []
    for label_observations in by_label.values():
        reference = [
            observation for observation in label_observations
            if str(observation.camera_id) == str(reference_camera_id)
        ]
        others = {}
        for observation in label_observations:
            camera_id = str(observation.camera_id)
            if camera_id == str(reference_camera_id):
                continue
            others.setdefault(camera_id, []).append(observation)
        if not reference or not others:
            continue
        max_age_ns = int(float(max_pair_age_s) * 1e9)
        for camera_id, camera_observations in others.items():
            for observation in camera_observations:
                nearest = min(
                    reference,
                    key=lambda item: abs(
                        int(item.capture_ns) - int(observation.capture_ns)))
                age_ns = abs(
                    int(nearest.capture_ns) - int(observation.capture_ns))
                if age_ns > max_age_ns:
                    continue
                residual = (
                    np.asarray(observation.position_base, dtype=np.float64)
                    - np.asarray(nearest.position_base, dtype=np.float64))
                residuals.append((camera_id, residual, age_ns))
    return residuals


def offset_statistics(residuals: Iterable[Tuple]):
    """
    Return ``{camera_id: {'mean', 'std', 'count', 'sigma_mean'}}``.

    ``mean`` is the offset to correct that camera by (negative of the measured
    residual of the camera relative to the reference). ``std`` is the
    per-sample spread and ``sigma_mean`` the 1σ uncertainty of the mean. With
    only two views the absolute split is unidentifiable, so the reference view
    is left uncorrected by construction.
    """
    grouped = {}
    for camera_id, residual, _ in residuals:
        grouped.setdefault(str(camera_id), []).append(
            np.asarray(residual, dtype=np.float64).reshape(3))
    stats = {}
    for camera_id, values in grouped.items():
        array = np.asarray(values, dtype=np.float64)
        count = int(array.shape[0])
        # Measured residual = z_camera - z_reference, so the correction added to
        # the camera is its negative.
        mean = -array.mean(axis=0) if count else np.zeros(3)
        std = array.std(axis=0, ddof=1) if count > 1 else np.zeros(3)
        sigma_mean = std / np.sqrt(max(1, count))
        stats[camera_id] = {
            'mean': mean,
            'std': std,
            'sigma_mean': sigma_mean,
            'count': count,
        }
    return stats
