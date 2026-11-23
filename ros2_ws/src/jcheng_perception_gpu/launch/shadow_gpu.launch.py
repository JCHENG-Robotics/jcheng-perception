"""Launch a shadow GPU detector alongside production.

The shadow node reads the same uplink images but publishes to
``/perception/detections_shadow/*``. The edge ignores those topics, so a
candidate model can be evaluated without touching the fused output.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(name):
    """Return the installed config path."""
    return os.path.join(
        get_package_share_directory('jcheng_perception_gpu'), 'config', name)


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpu_config',
            default_value=_config('gpu_shadow.example.yaml')),
        DeclareLaunchArgument('debug_publish', default_value='false'),
        DeclareLaunchArgument('queue_size', default_value='0'),
        DeclareLaunchArgument(
            'status_topic', default_value='/perception/gpu_status_shadow'),
        Node(
            package='jcheng_perception_gpu',
            executable='perception_gpu',
            name='jcheng_perception_gpu_shadow',
            output='screen',
            parameters=[{
                'gpu_config': LaunchConfiguration('gpu_config'),
                'debug_publish': LaunchConfiguration('debug_publish'),
                'queue_size': LaunchConfiguration('queue_size'),
                'status_topic': LaunchConfiguration('status_topic'),
            }],
        ),
    ])
