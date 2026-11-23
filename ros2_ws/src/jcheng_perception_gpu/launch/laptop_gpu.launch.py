"""Launch the GPU detector node on the laptop."""

import os
import sys

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
            'gpu_config', default_value=_config('gpu.yaml')),
        DeclareLaunchArgument('debug_publish', default_value='false'),
        DeclareLaunchArgument('queue_size', default_value='0'),
        DeclareLaunchArgument(
            'python_executable', default_value=sys.executable,
            description='Python interpreter for the detector node; point it '
                        'at a venv with Ultralytics/ONNX Runtime.'),
        Node(
            package='jcheng_perception_gpu',
            executable='perception_gpu',
            name='jcheng_perception_gpu',
            output='screen',
            prefix=[LaunchConfiguration('python_executable'), ' '],
            parameters=[{
                'gpu_config': LaunchConfiguration('gpu_config'),
                'debug_publish': LaunchConfiguration('debug_publish'),
                'queue_size': LaunchConfiguration('queue_size'),
            }],
        ),
    ])
