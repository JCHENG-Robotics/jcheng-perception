"""One-machine end-to-end demo: synthetic cameras + edge + mock GPU detector.

This exercises the distributed data path without hardware, a GPU or a model
file. Open the debug topics in rqt_image_view to see the red target.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _edge_config(name):
    """Return the installed edge config path."""
    return os.path.join(
        get_package_share_directory('jcheng_perception_edge'), 'config', name)


def _gpu_config(name):
    """Return the installed GPU config path."""
    return os.path.join(
        get_package_share_directory('jcheng_perception_gpu'), 'config', name)


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        Node(
            package='jcheng_perception_edge',
            executable='synthetic_scene_publisher',
            name='synthetic_scene',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 400,
                'rate_hz': 10.0,
                'depth_noise_m': 0.0008,
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='perception_edge',
            name='jcheng_perception_edge',
            output='screen',
            parameters=[{
                'cameras_config': _edge_config('cameras_synthetic.yaml'),
                'fusion_config': _edge_config('fusion_demo.yaml'),
                'frames_config': _edge_config('frames.yaml'),
                'timing_log': True,
            }],
        ),
        Node(
            package='jcheng_perception_gpu',
            executable='perception_gpu',
            name='jcheng_perception_gpu',
            output='screen',
            parameters=[{
                'gpu_config': _gpu_config('gpu_mock.yaml'),
                'debug_publish': False,
            }],
        ),
    ])
