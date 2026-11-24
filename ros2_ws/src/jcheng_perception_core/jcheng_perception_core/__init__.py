"""
JCHENG distributed perception core.

Pure-Python, ROS-free implementation of timestamp matching, depth
back-projection, target association, distance-weighted two-view fusion and
Kalman tracking. The HTTP/ROS adapters live in the edge and GPU packages.
"""

from .backproject import build_observation
from .config import (
    CameraSpec,
    FusionConfig,
    load_camera_specs,
    load_fusion_config,
    load_gpu_config,
    load_static_frames,
)
from .depth import (
    border_occlusion_ratio,
    depth_sigma_m,
    depth_valid_fraction,
    lateral_sigma_m,
    sample_bbox_depth,
    sample_depth,
)
from .geometry import (
    deproject_pixel,
    invert_transform,
    make_transform,
    matrix_from_quaternion,
    object_yaw_from_points,
    plane_pose_from_points,
    project_point,
    quaternion_from_matrix,
    quaternion_slerp,
    quaternion_yaw,
    transform_point,
    transform_points,
    yaw_quaternion,
)
from .kalman import ConstantVelocityKalman
from .pipeline import PerceptionPipeline
from .time_buffer import (
    PoseRingBuffer,
    TimestampRingBuffer,
    link_stalled,
    uplink_allowed,
)
from .tracking import Track, TrackManager
from .types import (
    CameraHealth,
    Detection,
    DepthFrame,
    FrameMeta,
    FusedTarget,
    Keypoint,
    Phase,
    PipelineStatus,
    Role,
    TargetObservation,
    TrackStatus,
)
from .weighting import fuse_observations, phase_for_observations, view_factor
from .view_bias import (
    ViewBias,
    ViewBiasModel,
    offset_statistics,
    paired_offsets,
)

__all__ = [
    'CameraHealth',
    'CameraSpec',
    'ConstantVelocityKalman',
    'Detection',
    'DepthFrame',
    'FrameMeta',
    'FusedTarget',
    'FusionConfig',
    'Keypoint',
    'PerceptionPipeline',
    'Phase',
    'PipelineStatus',
    'PoseRingBuffer',
    'Role',
    'TargetObservation',
    'TimestampRingBuffer',
    'Track',
    'TrackManager',
    'TrackStatus',
    'ViewBias',
    'ViewBiasModel',
    'border_occlusion_ratio',
    'build_observation',
    'depth_sigma_m',
    'depth_valid_fraction',
    'deproject_pixel',
    'fuse_observations',
    'invert_transform',
    'lateral_sigma_m',
    'link_stalled',
    'load_camera_specs',
    'load_fusion_config',
    'load_gpu_config',
    'load_static_frames',
    'make_transform',
    'matrix_from_quaternion',
    'object_yaw_from_points',
    'offset_statistics',
    'paired_offsets',
    'phase_for_observations',
    'plane_pose_from_points',
    'project_point',
    'quaternion_from_matrix',
    'quaternion_slerp',
    'quaternion_yaw',
    'sample_bbox_depth',
    'sample_depth',
    'transform_point',
    'transform_points',
    'uplink_allowed',
    'view_factor',
    'yaw_quaternion',
]
