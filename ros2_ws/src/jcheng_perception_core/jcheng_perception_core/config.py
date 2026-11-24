"""
Configuration loading and validation for the perception pipeline.

The same camera definitions are used by the Pi 5 edge node and the GPU node so
a topic rename cannot silently desynchronise the two halves. Unknown keys are
ignored for forward compatibility; missing keys fall back to the defaults here.
"""

import dataclasses
from typing import Dict, List, Optional

import numpy as np
import yaml

from .types import Phase, Role

DEFAULT_PHASE_PRIORS = {
    Phase.SEARCH: {Role.EXTERNAL: 1.0, Role.WRIST: 1.0},
    Phase.COARSE: {Role.EXTERNAL: 1.3, Role.WRIST: 0.6},
    Phase.ACQUIRE: {Role.EXTERNAL: 1.0, Role.WRIST: 1.0},
    Phase.FINE: {Role.EXTERNAL: 0.35, Role.WRIST: 3.0},
    Phase.DROP_RECHECK: {Role.EXTERNAL: 3.0, Role.WRIST: 0.0},
}


def _as_role(value):
    """Parse a role string into the enum."""
    text = str(value).strip().lower()
    if text == 'external':
        return Role.EXTERNAL
    if text == 'wrist':
        return Role.WRIST
    raise ValueError('unknown camera role: {}'.format(value))


def _as_phase(value):
    """Parse a phase name into the enum."""
    if isinstance(value, Phase):
        return value
    text = str(value).strip().upper()
    try:
        return Phase[text]
    except KeyError as error:
        raise ValueError('unknown phase: {}'.format(value)) from error


def _float_matrix(value, shape, name):
    """Validate an optional numeric matrix."""
    if value is None:
        return None
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != shape:
        raise ValueError(
            '{} must have shape {}, got {}'.format(name, shape, matrix.shape))
    return matrix


@dataclasses.dataclass
class CameraSpec:
    """Static per-camera configuration shared by edge and GPU."""

    camera_id: str
    role: Role
    rgb_topic: str
    depth_topic: str
    camera_info_topic: str
    optical_frame: str
    uplink_topic: str
    detection_topic: str
    rgb_compressed: bool = False
    depth_compressed: bool = False
    depth_scale: float = 0.001
    depth_is_metric: bool = False
    jpeg_quality: int = 78
    max_uplink_fps: float = 15.0
    max_detection_age_ms: float = 350.0
    frame_match_tolerance_ms: float = 60.0
    depth_match_tolerance_ms: float = 100.0
    pose_match_tolerance_ms: float = 80.0
    frame_buffer_seconds: float = 5.0
    depth_buffer_seconds: float = 5.0
    pose_buffer_seconds: float = 5.0
    max_buffer_items: int = 256
    pixel_sigma_px: float = 1.5
    depth_noise_quadratic: float = 0.0025
    depth_noise_floor_m: float = 0.0015
    min_depth_m: float = 0.05
    max_depth_m: float = 10.0
    min_valid_depth_fraction: float = 0.10
    min_range_m: float = 0.05
    max_range_m: float = 3.0
    range_penalty_width_m: float = 0.25
    keypoint_min_confidence: float = 0.25
    max_keypoints: int = 8
    keypoint_axis_pairs: List[List[int]] = dataclasses.field(
        default_factory=list)
    intrinsics: Optional[np.ndarray] = None
    distortion: Optional[np.ndarray] = None
    static_extrinsic: Optional[np.ndarray] = None
    pose_static: bool = False

    def __post_init__(self):
        """Validate numeric fields and normalise matrices."""
        if not self.camera_id:
            raise ValueError('camera_id must not be empty')
        if not self.optical_frame:
            raise ValueError(
                '{}: optical_frame must not be empty'.format(
                    self.camera_id))
        if self.depth_scale <= 0.0:
            raise ValueError(
                '{}: depth_scale must be positive'.format(self.camera_id))
        if not 0 <= self.jpeg_quality <= 100:
            raise ValueError(
                '{}: jpeg_quality must be within 0..100'.format(
                    self.camera_id))
        if self.min_depth_m <= 0.0 or self.max_depth_m <= self.min_depth_m:
            raise ValueError(
                '{}: invalid depth range'.format(self.camera_id))
        if self.min_range_m <= 0.0 or self.max_range_m <= self.min_range_m:
            raise ValueError(
                '{}: invalid working range'.format(self.camera_id))
        self.intrinsics = _float_matrix(
            self.intrinsics, (3, 3),
            '{} intrinsics'.format(self.camera_id))
        self.distortion = _float_matrix(
            self.distortion, (5,),
            '{} distortion'.format(self.camera_id))
        self.static_extrinsic = _float_matrix(
            self.static_extrinsic, (4, 4),
            '{} static_extrinsic'.format(self.camera_id))
        if self.static_extrinsic is not None:
            rotation = self.static_extrinsic[:3, :3]
            if not np.allclose(
                    rotation.T @ rotation, np.eye(3), atol=1e-5):
                raise ValueError(
                    '{}: static_extrinsic rotation is not orthonormal'.format(
                        self.camera_id))
        self.keypoint_axis_pairs = [
            [int(pair[0]), int(pair[1])]
            for pair in self.keypoint_axis_pairs
        ]

    @property
    def max_buffer_items_config(self):
        """Return the ring item cap."""
        return max(32, int(self.max_buffer_items))


@dataclasses.dataclass
class FusionConfig:
    """Parameters for association, filtering and two-view fusion."""

    target_frame: str = 'base_link'
    coalesce_ms: float = 30.0
    confirm_hits: int = 3
    tentative_timeout_s: float = 0.6
    coast_after_s: float = 0.6
    lost_after_s: float = 1.8
    association_gate_m: float = 0.08
    association_gate_growth_m: float = 0.12
    association_gate_per_second: float = 0.10
    association_label_agnostic: bool = False
    enable_track_merging: bool = True
    merge_gate_m: float = 0.06
    merge_gate_overlap_m: float = 0.02
    merge_sigma_scale: float = 3.0
    kalman_accel_noise: float = 0.35
    measurement_floor_m: float = 0.0008
    max_position_variance_m2: float = 0.04
    # Velocity is only observable within one view: two views land on different
    # physical points of a 3-D object, so a view change must not be read as
    # motion. Only successive samples of the same camera become a velocity
    # measurement; the age gate rejects a stale baseline.
    initial_velocity_sigma_mps: float = 0.25
    velocity_sample_max_age_s: float = 1.5
    # Two observations can share (or nearly share) a capture stamp when a
    # frame is matched to the same depth sample twice. Dividing a millimetre
    # difference by a few hundredths of a second fabricates tens of mm/s, so a
    # velocity baseline must span at least this interval.
    velocity_min_dt_s: float = 0.10
    # Publishing and association prediction clamp velocity extrapolation; a
    # millimetre-velocity estimate integrated for a second is decimetres.
    max_extrapolation_s: float = 0.35
    # Per-track residual view offset (external minus wrist, after the global
    # per-view correction). It absorbs the bbox-centre measurement-point
    # difference for the tracked object. The prior is deliberately wide (20 mm)
    # until the first time-matched pair establishes it.
    enable_view_offset: bool = True
    view_offset_alpha: float = 0.35
    view_offset_pair_max_age_s: float = 1.0
    view_offset_sigma_m: float = 0.020
    # Prior offset uncertainty decays with the number of learned pairs and
    # bottoms out at the residual model error (inter-view motion, object shape).
    view_offset_floor_sigma_m: float = 0.004
    occlusion_penalty: float = 0.75
    min_view_weight: float = 0.02
    fine_distance_m: float = 0.55
    fine_wrist_hits: int = 2
    orientation_max_tilt_deg: float = 35.0
    table_clearance_m: float = 0.008
    orientation_sigma_rad: float = 0.20
    publish_rate_hz: float = 30.0
    pose_sample_hz: float = 100.0
    status_rate_hz: float = 1.0
    recheck_duration_s: float = 3.0
    stale_gpu_s: float = 1.0
    phase_priors: Dict[Phase, Dict[Role, float]] = dataclasses.field(
        default_factory=lambda: {
            phase: dict(weights)
            for phase, weights in DEFAULT_PHASE_PRIORS.items()
        })

    def prior(self, phase, role):
        """Return the configured phase/role prior."""
        phase = _as_phase(phase) if isinstance(phase, str) else phase
        return float(self.phase_priors.get(
            phase, DEFAULT_PHASE_PRIORS[phase]).get(role, 1.0))


def load_yaml(path):
    """Load a YAML document, returning an empty mapping for an empty file."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data or {}


def load_camera_specs(path):
    """Load ``cameras.yaml`` into an ordered ``{camera_id: CameraSpec}``."""
    document = load_yaml(path)
    entries = document.get('cameras')
    if not isinstance(entries, dict) or not entries:
        raise ValueError('{}: cameras must be a non-empty mapping'.format(path))
    specs = {}
    for camera_id, entry in entries.items():
        if not isinstance(entry, dict):
            raise ValueError(
                '{}: camera {} must be a mapping'.format(path, camera_id))
        values = dict(entry)
        values['camera_id'] = str(camera_id)
        values['role'] = _as_role(values.get('role', 'external'))
        for required in (
                'rgb_topic', 'depth_topic', 'camera_info_topic',
                'optical_frame'):
            if not values.get(required):
                raise ValueError(
                    '{}: camera {} is missing {}'.format(
                        path, camera_id, required))
        values.setdefault(
            'uplink_topic',
            '/perception/uplink/{}/image_compressed'.format(camera_id))
        values.setdefault(
            'detection_topic',
            '/perception/detections/{}'.format(camera_id))
        known = {field.name for field in dataclasses.fields(CameraSpec)}
        filtered = {key: value for key, value in values.items()
                    if key in known}
        specs[str(camera_id)] = CameraSpec(**filtered)
    if len({spec.role for spec in specs.values()}) < 2:
        raise ValueError(
            '{}: both an external and a wrist camera are required'.format(path))
    return specs


def load_fusion_config(path):
    """Load ``fusion.yaml`` into a :class:`FusionConfig`."""
    document = load_yaml(path)
    known = {field.name for field in dataclasses.fields(FusionConfig)}
    filtered = {key: value for key, value in document.items()
                if key in known and key != 'phase_priors'}
    config = FusionConfig(**filtered)
    priors = document.get('phase_priors')
    if priors:
        parsed = {}
        for phase_name, weights in priors.items():
            phase = _as_phase(phase_name)
            parsed[phase] = {
                _as_role(role): float(value)
                for role, value in (weights or {}).items()
            }
        for phase in Phase:
            parsed.setdefault(phase, dict(DEFAULT_PHASE_PRIORS[phase]))
        config.phase_priors = parsed
    if config.confirm_hits < 1:
        raise ValueError('confirm_hits must be at least 1')
    if config.lost_after_s <= config.coast_after_s:
        raise ValueError('lost_after_s must exceed coast_after_s')
    if config.publish_rate_hz <= 0.0:
        raise ValueError('publish_rate_hz must be positive')
    if config.initial_velocity_sigma_mps < 0.0:
        raise ValueError('initial_velocity_sigma_mps must not be negative')
    if config.velocity_sample_max_age_s <= 0.0:
        raise ValueError('velocity_sample_max_age_s must be positive')
    if config.velocity_min_dt_s <= 0.0:
        raise ValueError('velocity_min_dt_s must be positive')
    if config.velocity_min_dt_s > config.velocity_sample_max_age_s:
        raise ValueError(
            'velocity_min_dt_s must not exceed velocity_sample_max_age_s')
    if config.max_extrapolation_s < 0.0:
        raise ValueError('max_extrapolation_s must not be negative')
    if not 0.0 <= config.view_offset_alpha <= 1.0:
        raise ValueError('view_offset_alpha must be within 0..1')
    if config.view_offset_pair_max_age_s < 0.0:
        raise ValueError('view_offset_pair_max_age_s must not be negative')
    if config.view_offset_sigma_m < 0.0:
        raise ValueError('view_offset_sigma_m must not be negative')
    if config.view_offset_floor_sigma_m < 0.0:
        raise ValueError('view_offset_floor_sigma_m must not be negative')
    return config


def load_gpu_config(path):
    """Load the GPU detector configuration.

    Returns ``{'node': dict, 'cameras': {camera_id: dict}}`` with defaults
    applied to every camera model.
    """
    document = load_yaml(path)
    node = dict(document.get('node') or {})
    node.setdefault('stats_interval_s', 2.0)
    node.setdefault('debug_publish', False)
    node.setdefault('debug_interval_s', 1.0)
    node.setdefault('queue_size', 2)
    cameras = document.get('cameras')
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError('{}: cameras must be a non-empty mapping'.format(path))
    resolved = {}
    for camera_id, entry in cameras.items():
        if not isinstance(entry, dict):
            raise ValueError(
                '{}: camera {} must be a mapping'.format(path, camera_id))
        model = dict(entry.get('model') or {})
        model.setdefault('backend', 'ultralytics')
        model.setdefault('task', 'detect')
        model.setdefault('imgsz', 640)
        model.setdefault('conf', 0.35)
        model.setdefault('iou', 0.45)
        model.setdefault('device', '0')
        model.setdefault('half', False)
        model.setdefault('max_detections', 20)
        model.setdefault('labels', [])
        model.setdefault('keypoints', False)
        model.setdefault('hsv_lower', [0, 120, 80])
        model.setdefault('hsv_upper', [10, 255, 255])
        model.setdefault('hsv_lower2', [170, 120, 80])
        model.setdefault('hsv_upper2', [180, 255, 255])
        if model['backend'] == 'ultralytics' and not model.get('path'):
            raise ValueError(
                '{}: camera {} needs model.path'.format(path, camera_id))
        resolved[str(camera_id)] = {
            'input_topic': entry.get(
                'input_topic',
                '/perception/uplink/{}/image_compressed'.format(camera_id)),
            'detections_topic': entry.get(
                'detections_topic',
                '/perception/detections/{}'.format(camera_id)),
            'debug_topic': entry.get(
                'debug_topic',
                '/perception/debug/{}'.format(camera_id)),
            'hot_reload': bool(entry.get('hot_reload', True)),
            'model': model,
        }
    return {'node': node, 'cameras': resolved}


def load_static_frames(path):
    """Load optional static base<-camera fallbacks from ``frames.yaml``."""
    if not path:
        return {}
    document = load_yaml(path)
    frames = {}
    for camera_id, entry in (document.get('static_fallbacks') or {}).items():
        matrix = _float_matrix(
            entry.get('transform'), (4, 4),
            '{} static fallback'.format(camera_id))
        if matrix is not None:
            frames[str(camera_id)] = matrix
    return frames
