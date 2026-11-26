"""
Backend boundary for ore instance segmentation.

Every backend consumes one BGR image and returns disjoint boolean instance
masks. Backends never read datasets, write files, publish messages or touch a
device; those responsibilities stay in the pipeline and CLI.
"""

import dataclasses
from typing import List

from ..types import InstanceMask


@dataclasses.dataclass(frozen=True)
class BackendInfo:
    """Static description recorded in every scene-state record."""

    backend_id: str
    version: str
    family: str
    calibrated: bool
    description: str

    def to_dict(self):
        """Return a JSON-safe description."""
        return {
            'id': self.backend_id,
            'version': self.version,
            'family': self.family,
            'calibrated': bool(self.calibrated),
            'description': self.description,
        }


class SegmentationBackend:
    """Base class for offline instance-segmentation backends."""

    info = BackendInfo(
        backend_id='abstract',
        version='0',
        family='none',
        calibrated=False,
        description='abstract backend',
    )

    def segment(self, image_bgr, config, scene_id):
        """Return a list of disjoint :class:`InstanceMask` objects."""
        raise NotImplementedError
