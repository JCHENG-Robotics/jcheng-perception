"""
Versioned, transport-independent scene datatypes.

The names here are deliberately narrow. ``visible_centroid_px`` is the centroid
of the observed mask only; it is never a claim about the object's true centre.
``visible_extent_px`` is the 2-D extent of the observed mask, not the full
object size. Fields that S1 cannot observe are listed in ``UNKNOWNS`` and are
never synthesised.
"""

import dataclasses
import hashlib
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

SCENE_SCHEMA_VERSION = 'jcheng.scene_state.v1'
PREDICTION_SCHEMA_VERSION = 'jcheng.ore_prediction.v1'
DATASET_SCHEMA_VERSION = 'jcheng.ore_dataset.v1'
EVALUATION_SCHEMA_VERSION = 'jcheng.ore_evaluation.v1'

SEMANTIC_LABEL = 'ore'

# Fields that are not observed in the S1 RGB-only pipeline. They stay explicit
# so no consumer can mistake their absence for a zero or a measured value.
UNKNOWNS = [
    'occlusion_direction',
    'true_centroid',
    'full_extent',
    'volume',
    'mass',
    'material_composition',
    'ore_grade',
    'economic_value',
]

SCENE_FIELD_STATUS = {
    'mask': 'algorithm_estimate',
    'bbox_xyxy': 'algorithm_estimate',
    'detection_confidence': 'algorithm_estimate',
    'visible_centroid_px': 'algorithm_estimate',
    'visible_extent_px': 'algorithm_estimate',
    'adjacency': 'algorithm_estimate',
    'occlusion_direction': 'unknown',
    'true_centroid': 'unknown',
    'full_extent': 'unknown',
    'volume': 'unknown',
    'mass': 'unknown',
    'material_composition': 'unknown',
    'ore_grade': 'unknown',
    'economic_value': 'unknown',
}

# Structured failure codes. The CLI maps these to non-zero exit codes.
ERROR_INPUT_IMAGE = 'INPUT_IMAGE_UNREADABLE'
ERROR_INPUT_DATASET = 'INPUT_DATASET_INVALID'
ERROR_CONFIG = 'CONFIG_INVALID'
ERROR_BACKEND = 'BACKEND_FAILURE'
ERROR_MASK_SHAPE = 'MASK_SHAPE_MISMATCH'
ERROR_MASK_OVERLAP = 'MASK_OVERLAP'
ERROR_INSTANCE = 'INSTANCE_INVALID'
ERROR_GROUND_TRUTH = 'GROUND_TRUTH_MISSING'
ERROR_EVALUATION = 'EVALUATION_INVALID'


def sha256_bytes(payload):
    """Return the SHA-256 hex digest of a byte string."""
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    """Return the SHA-256 hex digest of a file, or ``None`` if unreadable."""
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError:
        return None
    return digest.hexdigest()


@dataclasses.dataclass
class SceneError:
    """One structured, non-fatal or fatal scene-processing error."""

    code: str
    message: str
    stage: str = 'unknown'
    recoverable: bool = False
    context: Dict[str, Any] = dataclasses.field(default_factory=dict)

    def to_dict(self):
        """Return a JSON-safe representation."""
        return {
            'code': str(self.code),
            'message': str(self.message),
            'stage': str(self.stage),
            'recoverable': bool(self.recoverable),
            'context': dict(self.context),
        }


@dataclasses.dataclass
class InstanceMask:
    """One observed instance mask plus its visible 2-D evidence."""

    instance_id: str
    mask: np.ndarray
    semantic_label: str = SEMANTIC_LABEL
    score: float = 0.0
    confidence_kind: str = 'heuristic_uncalibrated'
    mask_index: int = 0
    mask_path: Optional[str] = None
    mask_sha256: Optional[str] = None
    attributes: Dict[str, Any] = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        """Normalise and validate the mask and score."""
        if not isinstance(self.instance_id, str) or not self.instance_id:
            raise ValueError('instance_id must be a non-empty string')
        mask = np.asarray(self.mask)
        if mask.ndim != 2:
            raise ValueError('instance mask must be 2-D')
        mask = mask.astype(bool, copy=False)
        if not np.any(mask):
            raise ValueError('instance mask must contain at least one pixel')
        score = float(self.score)
        if not np.isfinite(score) or score < 0.0 or score > 1.0:
            raise ValueError('instance score must be finite and within [0, 1]')
        if int(self.mask_index) < 0:
            raise ValueError('mask_index must be non-negative')
        self.mask = mask
        self.score = score
        rows, columns = np.nonzero(mask)
        self._area_px = int(rows.size)
        self._bbox_xyxy = (
            int(columns.min()), int(rows.min()),
            int(columns.max()), int(rows.max()),
        )
        self._visible_centroid_px = (
            float(columns.mean()), float(rows.mean()))

    @property
    def height(self):
        """Return the mask height in pixels."""
        return int(self.mask.shape[0])

    @property
    def width(self):
        """Return the mask width in pixels."""
        return int(self.mask.shape[1])

    @property
    def area_px(self):
        """Return the number of observed mask pixels."""
        return self._area_px

    @property
    def bbox_xyxy(self):
        """Return the inclusive pixel bounding box of the observed mask."""
        return self._bbox_xyxy

    @property
    def visible_centroid_px(self):
        """Return the centroid of the observed mask in pixel coordinates."""
        return self._visible_centroid_px

    @property
    def visible_extent_px(self):
        """Return the observed mask width and height in pixels."""
        x0, y0, x1, y1 = self.bbox_xyxy
        return (int(x1 - x0 + 1), int(y1 - y0 + 1))

    def summary(self):
        """Return the JSON-safe scene-state representation of this instance."""
        x0, y0, x1, y1 = self.bbox_xyxy
        cx, cy = self.visible_centroid_px
        return {
            'instance_id': self.instance_id,
            'semantic_label': str(self.semantic_label),
            'detection_confidence': float(self.score),
            'confidence_kind': str(self.confidence_kind),
            'bbox_xyxy': [x0, y0, x1, y1],
            'mask_path': self.mask_path,
            'mask_sha256': self.mask_sha256,
            'mask_index': int(self.mask_index),
            'mask_area_px': self.area_px,
            'visible_centroid_px': [round(cx, 3), round(cy, 3)],
            'visible_extent_px': list(self.visible_extent_px),
            'geometry': {
                'status': 'not_computed_in_s1',
                'reason': 'rgb_only_pipeline',
            },
            'attributes': dict(self.attributes),
        }


@dataclasses.dataclass
class Relation:
    """One evidence-backed relation between two observed instances."""

    subject_id: str
    predicate: str
    object_id: str
    confidence: float
    evidence: Dict[str, Any] = dataclasses.field(default_factory=dict)
    source: str = '2d_mask'

    def to_dict(self):
        """Return a JSON-safe representation."""
        return {
            'subject_id': str(self.subject_id),
            'predicate': str(self.predicate),
            'object_id': str(self.object_id),
            'confidence': float(self.confidence),
            'evidence': dict(self.evidence),
            'source': str(self.source),
        }


@dataclasses.dataclass
class Validity:
    """Explicit validity, missing-field and failure information."""

    valid: bool = True
    empty_scene: bool = False
    missing_fields: List[str] = dataclasses.field(default_factory=list)
    failure_reason: Optional[str] = None
    unknowns: List[str] = dataclasses.field(
        default_factory=lambda: list(UNKNOWNS))

    def to_dict(self):
        """Return a JSON-safe representation."""
        return {
            'valid': bool(self.valid),
            'empty_scene': bool(self.empty_scene),
            'missing_fields': list(self.missing_fields),
            'failure_reason': self.failure_reason,
            'unknowns': list(self.unknowns),
        }


@dataclasses.dataclass
class SceneState:
    """A versioned observation of one scene at one timestamp."""

    scene_id: str
    timestamp_ns: int
    frame_id: str
    image_width: int
    image_height: int
    instances: List[InstanceMask] = dataclasses.field(default_factory=list)
    relations: List[Relation] = dataclasses.field(default_factory=list)
    observation_source: Dict[str, Any] = dataclasses.field(
        default_factory=dict)
    backend: Dict[str, Any] = dataclasses.field(default_factory=dict)
    image_path: Optional[str] = None
    image_sha256: Optional[str] = None
    errors: List[SceneError] = dataclasses.field(default_factory=list)
    timing: Dict[str, float] = dataclasses.field(default_factory=dict)
    disjoint_verified: bool = False
    schema_version: str = SCENE_SCHEMA_VERSION
    field_status: Dict[str, str] = dataclasses.field(
        default_factory=lambda: dict(SCENE_FIELD_STATUS))

    def __post_init__(self):
        """Validate dimensions and check the mask partition invariant."""
        if not isinstance(self.scene_id, str) or not self.scene_id:
            raise ValueError('scene_id must be a non-empty string')
        if int(self.image_width) <= 0 or int(self.image_height) <= 0:
            raise ValueError('image dimensions must be positive')
        if int(self.timestamp_ns) < 0:
            raise ValueError('timestamp_ns must be non-negative')
        seen = set()
        for instance in self.instances:
            if not isinstance(instance, InstanceMask):
                raise TypeError('instances must be InstanceMask objects')
            if instance.instance_id in seen:
                raise ValueError('duplicate instance_id: %s' %
                                 instance.instance_id)
            seen.add(instance.instance_id)
            if instance.height != int(self.image_height) or \
                    instance.width != int(self.image_width):
                raise ValueError('instance mask shape does not match scene')
        if self.instances and not self.disjoint_verified:
            if len(self.instances) <= 64:
                union = self.instances[0].mask.copy()
                for instance in self.instances[1:]:
                    if np.any(union & instance.mask):
                        raise ValueError(
                            ERROR_MASK_OVERLAP +
                            ': instance masks must not overlap')
                    union |= instance.mask
            else:
                coverage = np.zeros(
                    (int(self.image_height), int(self.image_width)),
                    dtype=np.uint8)
                for instance in self.instances:
                    coverage += instance.mask.astype(np.uint8)
                if int(coverage.max()) > 1:
                    raise ValueError(
                        ERROR_MASK_OVERLAP +
                        ': instance masks must not overlap')

    @property
    def empty_scene(self):
        """Return True when the backend observed no instance."""
        return len(self.instances) == 0

    def validity(self):
        """Return the aggregate validity record for this scene."""
        failure = None
        missing = []
        valid = True
        fatal = [error for error in self.errors if not error.recoverable]
        if fatal:
            valid = False
            failure = fatal[0].code
        if self.image_width <= 0 or self.image_height <= 0:
            valid = False
            missing.append('image_size')
        if not self.instances and not fatal:
            # An empty scene is a valid observation, not a failure.
            pass
        if not self.backend:
            missing.append('backend')
        return Validity(
            valid=valid,
            empty_scene=self.empty_scene,
            missing_fields=missing,
            failure_reason=failure,
            unknowns=list(UNKNOWNS),
        )

    def to_dict(self):
        """Return the JSON-safe scene-state record."""
        instances = [instance.summary() for instance in self.instances]
        return {
            'schema_version': self.schema_version,
            'scene_id': self.scene_id,
            'timestamp_ns': int(self.timestamp_ns),
            'frame_id': str(self.frame_id),
            'image': {
                'path': self.image_path,
                'sha256': self.image_sha256,
                'width': int(self.image_width),
                'height': int(self.image_height),
            },
            'observation_source': dict(self.observation_source),
            'backend': dict(self.backend),
            'instances': instances,
            'relations': [relation.to_dict() for relation in self.relations],
            'field_status': dict(self.field_status),
            'validity': self.validity().to_dict(),
            'errors': [error.to_dict() for error in self.errors],
            'timing': {key: float(value) for key, value in self.timing.items()},
        }
