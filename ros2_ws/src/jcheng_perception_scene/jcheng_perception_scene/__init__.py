"""
JCHENG offline ore scene understanding.

ROS-free by construction: this package contains no rclpy, DDS, device or
motion dependency. It produces advisory scene state and evaluation artifacts
only. S1 is RGB-only and does not estimate 3-D geometry, mass, material or ore
grade.
"""

from .association import (
    AssociationParams,
    associate_scene,
    evaluate_association,
    fuse_association,
    load_dual_view_dataset,
    read_association_jsonl,
    validate_association_record,
    write_association_jsonl,
)
from .config import DEFAULT_CONFIG, config_sha256, load_config, validate_config
from .dataset import import_coco_dataset, load_dataset, read_predictions
from .geometry import GeometryParams, visible_geometry
from .metrics import evaluate
from .relation_benchmark import (
    evaluate_relation_benchmark,
    generate_relation_benchmark,
)
from .relation_eval import RelationEvalParams, evaluate_predicted_relations
from .relations import (
    RelationParams,
    build_relation_graph,
    compare_relation_sets,
)
from .synthetic_dual import (
    DUAL_VIEW_DATASET_SCHEMA_VERSION,
    Camera,
    Ellipsoid,
    generate_dual_view_dataset,
    generate_dual_view_scene,
    render_view,
)
from .pipeline import run_dataset, run_unlabelled
from .sam2_tiles import (
    TileSet,
    TilesConfig,
    load_tiles_config,
    run_sam2_tiles,
)
from .scene import build_scene_state
from .scene_v2 import (
    SCENE_V2_SCHEMA_VERSION,
    build_scene_v2,
    geometry_for_mask,
    validate_scene_v2,
)
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
    'DUAL_VIEW_DATASET_SCHEMA_VERSION',
    'EVALUATION_SCHEMA_VERSION',
    'GeometryParams',
    'AssociationParams',
    'Camera',
    'Ellipsoid',
    'InstanceMask',
    'PREDICTION_SCHEMA_VERSION',
    'Relation',
    'RelationEvalParams',
    'RelationParams',
    'SCENE_SCHEMA_VERSION',
    'SCENE_V2_SCHEMA_VERSION',
    'SceneError',
    'SceneState',
    'TileSet',
    'TilesConfig',
    'UNKNOWNS',
    '__version__',
    'build_relation_graph',
    'build_scene_state',
    'build_scene_v2',
    'compare_relation_sets',
    'config_sha256',
    'associate_scene',
    'evaluate',
    'evaluate_association',
    'evaluate_predicted_relations',
    'evaluate_relation_benchmark',
    'fuse_association',
    'generate_dual_view_dataset',
    'generate_dual_view_scene',
    'generate_relation_benchmark',
    'geometry_for_mask',
    'import_coco_dataset',
    'load_config',
    'load_dataset',
    'load_dual_view_dataset',
    'load_tiles_config',
    'read_predictions',
    'read_association_jsonl',
    'render_view',
    'run_dataset',
    'run_sam2_tiles',
    'run_unlabelled',
    'validate_config',
    'validate_association_record',
    'validate_scene_v2',
    'visible_geometry',
    'write_association_jsonl',
]
