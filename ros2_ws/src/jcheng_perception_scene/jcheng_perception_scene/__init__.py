"""
JCHENG offline ore scene understanding.

ROS-free by construction: this package contains no rclpy, DDS, device or
motion dependency. It produces advisory scene state and evaluation artifacts
only. S1 is RGB-only and does not estimate 3-D geometry, mass, material or ore
grade.
"""

from .config import DEFAULT_CONFIG, config_sha256, load_config, validate_config
from .dataset import import_coco_dataset, load_dataset, read_predictions
from .geometry import GeometryParams, visible_geometry
from .metrics import evaluate
from .pipeline import run_dataset, run_unlabelled
from .sam2_tiles import (
    TileSet,
    TilesConfig,
    load_tiles_config,
    run_sam2_tiles,
)
from .scene import build_scene_state
from .types import (
    DATASET_SCHEMA_VERSION,
    EVALUATION_SCHEMA_VERSION,
    PREDICTION_SCHEMA_VERSION,
    SCENE_SCHEMA_VERSION,
    UNKNOWNS,
    InstanceMask,
    Relation,
    SceneError,
    SceneState,
)

__version__ = '0.1.0'

__all__ = [
    'DATASET_SCHEMA_VERSION',
    'DEFAULT_CONFIG',
    'EVALUATION_SCHEMA_VERSION',
    'GeometryParams',
    'InstanceMask',
    'PREDICTION_SCHEMA_VERSION',
    'Relation',
    'SCENE_SCHEMA_VERSION',
    'SceneError',
    'SceneState',
    'TileSet',
    'TilesConfig',
    'UNKNOWNS',
    '__version__',
    'build_scene_state',
    'config_sha256',
    'evaluate',
    'import_coco_dataset',
    'load_config',
    'load_dataset',
    'load_tiles_config',
    'read_predictions',
    'run_dataset',
    'run_sam2_tiles',
    'run_unlabelled',
    'validate_config',
    'visible_geometry',
]
