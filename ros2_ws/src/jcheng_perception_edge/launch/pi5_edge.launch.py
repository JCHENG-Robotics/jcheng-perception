"""Launch the Pi 5 edge node.

Run this on the robot (Pi 5). It reads camera topics, uplinks JPEG frames and
fuses GPU detections. It does not run YOLO.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(name):
    """Return the installed config path."""
    return os.path.join(
        get_package_share_directory('jcheng_perception_edge'), 'config', name)


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=_config('cameras.yaml')),
        DeclareLaunchArgument(
            'fusion_config', default_value=_config('fusion.yaml')),
        DeclareLaunchArgument(
            'frames_config', default_value=_config('frames.yaml')),
        DeclareLaunchArgument('target_frame', default_value=''),
        DeclareLaunchArgument('enable_uplink', default_value='true'),
        DeclareLaunchArgument('timing_log', default_value='false'),
        DeclareLaunchArgument('start_wrist_tf', default_value='true'),
        DeclareLaunchArgument(
            'wrist_tf_config', default_value=_config('wrist_tf.yaml')),
        Node(
            package='jcheng_perception_edge',
            executable='perception_edge',
            name='jcheng_perception_edge',
            output='screen',
            parameters=[{
                'cameras_config': LaunchConfiguration('cameras_config'),
                'fusion_config': LaunchConfiguration('fusion_config'),
                'frames_config': LaunchConfiguration('frames_config'),
                'target_frame': LaunchConfiguration('target_frame'),
                'enable_uplink': LaunchConfiguration('enable_uplink'),
                'timing_log': LaunchConfiguration('timing_log'),
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='wrist_tf',
            name='wrist_tf',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_wrist_tf')),
            parameters=[{
                'config_file': LaunchConfiguration('wrist_tf_config'),
            }],
        ),
    ])
