"""Test path setup for the ROS-free core package."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE = os.path.join(ROOT, 'ros2_ws', 'src', 'jcheng_perception_core')
TESTS = os.path.dirname(os.path.abspath(__file__))
for path in (CORE, TESTS):
    if path not in sys.path:
        sys.path.insert(0, path)
