"""
Datatypes shared by the edge, GPU and offline pipeline.

The core package has no ROS dependency. ROS message conversion lives in the
node packages; this module is the stable, testable boundary.
"""

import dataclasses
import enum
from typing import Dict, List, Optional, Tuple

import numpy as np


class Role(str, enum.Enum):
    """Semantic role of a camera in the manipulation task."""

    EXTERNAL = 'external'
    WRIST = 'wrist'

    @property
    def message_value(self):
        """Return the ROS message constant for this role."""
        return 1 if self is Role.EXTERNAL else 2


class Phase(enum.IntEnum):
    """Task phase; values match ``FusedTarget.msg`` constants."""

    SEARCH = 0
    COARSE = 1
    ACQUIRE = 2
    FINE = 3
    DROP_RECHECK = 4


class TrackStatus(enum.IntEnum):
    """Track lifecycle; values match ``FusedTarget.msg`` constants."""

    TENTATIVE = 0
    CONFIRMED = 1
    COASTING = 2
    LOST = 3


@dataclasses.dataclass
class DepthFrame:
    """One depth image plus the scale that converts it to metres."""

    data: np.ndarray
    scale_m: float = 0.001
    metric: bool = False
    stamp_ns: int = 0

    def __post_init__(self):
        """Validate the array and cache shape information."""
        if self.data.ndim != 2:
            raise ValueError('depth data must be a 2-D array')
        if not self.metric and self.scale_m <= 0.0:
            raise ValueError('depth scale must be positive')

    @property
    def height(self):
        """Return the image height in pixels."""
        return int(self.data.shape[0])

    @property
    def width(self):
        """Return the image width in pixels."""
        return int(self.data.shape[1])

    def patch_metres(self, u, v, radius):
        """Return a metric depth patch centred on ``(u, v)``."""
        height, width = self.data.shape
        u = int(round(u))
        v = int(round(v))
        x0 = max(0, u - radius)
        x1 = min(width, u + radius + 1)
        y0 = max(0, v - radius)
        y1 = min(height, v + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        patch = self.data[y0:y1, x0:x1]
        if self.metric:
            return np.asarray(patch, dtype=np.float64)
        return np.asarray(patch, dtype=np.float64) * float(self.scale_m)

    def roi_metres(self, x0, y0, x1, y1, stride=2):
        """Return an ``(N, 3)`` metric point cloud for a pixel rectangle.

        Rows with invalid (non-positive or non-finite) depth are removed.
        """
        height, width = self.data.shape
        x0 = int(max(0, min(width - 1, round(x0))))
        x1 = int(max(x0 + 1, min(width, round(x1))))
        y0 = int(max(0, min(height - 1, round(y0))))
        y1 = int(max(y0 + 1, min(height, round(y1))))
        step = max(1, int(stride))
        data = self.data[y0:y1:step, x0:x1:step]
        if self.metric:
            metres = np.asarray(data, dtype=np.float64)
        else:
            metres = np.asarray(data, dtype=np.float64) * float(self.scale_m)
        rows, columns = np.nonzero(np.isfinite(metres) & (metres > 0.0))
        if rows.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.column_stack([
            columns * step + x0,
            rows * step + y0,
            metres[rows, columns],
        ])


@dataclasses.dataclass
class FrameMeta:
    """Metadata for one acquisition, retained in the timestamp ring."""

    camera_id: str
    seq: int
    capture_ns: int
    received_ns: int
    width: int
    height: int
    frame_id: str
    camera_matrix: np.ndarray
    distortion: np.ndarray
    encoding: str = 'bgr8'


@dataclasses.dataclass
class Keypoint:
    """One 2-D keypoint in pixel coordinates."""

    x: float
    y: float
    confidence: float = 1.0


@dataclasses.dataclass
class Detection:
    """A detector output, independent of the transport."""

    label: str
    score: float
    bbox_xyxy: Tuple[float, float, float, float]
    class_id: int = 0
    keypoints: List[Keypoint] = dataclasses.field(default_factory=list)
    track_hint: int = 0


@dataclasses.dataclass
class TargetObservation:
    """One 3-D measurement of a target from one camera at one capture time."""

    camera_id: str
    role: Role
    label: str
    class_id: int
    capture_ns: int
    received_ns: int
    frame_seq: int
    frame_id_camera: str
    detection_score: float
    keypoint_score: float
    position_base: np.ndarray
    covariance_base: np.ndarray
    depth_m: float
    valid_depth_fraction: float
    occlusion_ratio: float
    orientation_base: Optional[np.ndarray] = None
    orientation_sigma_rad: float = 0.0
    pixel: Tuple[float, float] = (0.0, 0.0)
    view_weight: float = 0.0

    def __post_init__(self):
        """Normalise array fields to float64."""
        self.position_base = np.asarray(
            self.position_base, dtype=np.float64).reshape(3)
        self.covariance_base = np.asarray(
            self.covariance_base, dtype=np.float64).reshape(3, 3)
        if self.orientation_base is not None:
            self.orientation_base = np.asarray(
                self.orientation_base, dtype=np.float64).reshape(4)

    @property
    def is_occluded(self):
        """Return True when the depth support is too thin to be trusted."""
        return self.occlusion_ratio >= 0.5

    @property
    def variance_trace(self):
        """Return the mean per-axis position variance."""
        return float(np.trace(self.covariance_base)) / 3.0


@dataclasses.dataclass
class FusedTarget:
    """A fused, base-frame target ready for the planner."""

    target_id: str
    label: str
    class_id: int
    stamp_ns: int
    position_base: np.ndarray
    orientation_base: Optional[np.ndarray]
    covariance_6x6: np.ndarray
    confidence: float
    phase: Phase
    status: TrackStatus
    weights: Dict[str, float]
    depth_by_camera: Dict[str, float]
    occluded: bool
    recheck_required: bool
    source_cameras: List[str]
    age_s: float

    def __post_init__(self):
        """Normalise the array fields."""
        self.position_base = np.asarray(
            self.position_base, dtype=np.float64).reshape(3)
        self.covariance_6x6 = np.asarray(
            self.covariance_6x6, dtype=np.float64).reshape(6, 6)
        if self.orientation_base is not None:
            self.orientation_base = np.asarray(
                self.orientation_base, dtype=np.float64).reshape(4)


@dataclasses.dataclass
class CameraHealth:
    """Per-camera counters maintained by the edge node."""

    frames_received: int = 0
    frames_uplinked: int = 0
    frames_rate_limited: int = 0
    depth_received: int = 0
    camera_info_received: int = 0
    detections_received: int = 0
    detections_matched: int = 0
    depth_match_failures: int = 0
    tf_match_failures: int = 0
    frame_match_failures: int = 0
    stale_detections: int = 0
    encode_failures: int = 0
    tf_samples: int = 0
    tf_failures: int = 0
    last_frame_ns: int = 0
    last_detection_ns: int = 0
    last_latency_ms: float = 0.0
    last_depth_age_ms: float = 0.0
    last_frame_age_ms: float = 0.0
    mean_latency_ms: float = 0.0


@dataclasses.dataclass
class PipelineStatus:
    """Aggregated pipeline status for diagnostics and the status message."""

    phase: Phase = Phase.SEARCH
    active_tracks: int = 0
    confirmed_tracks: int = 0
    gpu_alive: bool = False
    last_detection_ns: int = 0
    notes: List[str] = dataclasses.field(default_factory=list)
