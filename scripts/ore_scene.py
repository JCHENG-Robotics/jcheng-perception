#!/usr/bin/env python3
"""Thin CLI entry point for the offline ore scene baseline."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE = os.path.join(ROOT, 'ros2_ws', 'src', 'jcheng_perception_core')
SCENE = os.path.join(ROOT, 'ros2_ws', 'src', 'jcheng_perception_scene')
for path in (CORE, SCENE):
    if path not in sys.path:
        sys.path.insert(0, path)

from jcheng_perception_scene.cli import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
