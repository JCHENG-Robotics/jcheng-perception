"""Split edge: one ingress process per camera plus a fusion process."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(name):
    """Return an installed config path."""
    return os.path.join(
        get_package_share_directory('jcheng_perception_edge'), 'config', name)


def _input(name):
    """Return a launch-config substitution for a config file."""
    return LaunchConfiguration(name)


def generate_launch_description():
    """Build the split-ingress launch description."""
    cameras = _input('cameras_config')
    fusion = _input('fusion_config')
    frames = _input('frames_config')
    view_bias = _input('view_bias_config')
    start_tf = IfCondition(LaunchConfiguration('start_tf'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=_config('cameras.yaml')),
        DeclareLaunchArgument(
            'fusion_config', default_value=_config('fusion_dual_hardware.yaml')),
        DeclareLaunchArgument(
            'frames_config', default_value=_config('frames.yaml')),
        DeclareLaunchArgument(
            'view_bias_config', default_value=_config('view_bias.yaml')),
        DeclareLaunchArgument('start_tf', default_value='false'),
        DeclareLaunchArgument(
            'wrist_tf_config', default_value=_config('wrist_tf.yaml')),
        DeclareLaunchArgument(
            'ext_tf_config', default_value=_config('ext_tf.yaml')),
        Node(
            package='jcheng_perception_edge',
            executable='perception_edge',
            name='perception_ingress_ext',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'camera_filter': 'd435i_ext',
                'publish_targets': False,
                'publish_status': False,
                'ingress_watchdog': True,
                'enable_uplink': True,
                'cameras_config': cameras,
                'fusion_config': fusion,
                'frames_config': frames,
                'view_bias_config': view_bias,
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='perception_edge',
            name='perception_ingress_wrist',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'camera_filter': 'aurora_wrist',
                'publish_targets': False,
                'publish_status': False,
                'ingress_watchdog': True,
                'enable_uplink': True,
                'cameras_config': cameras,
                'fusion_config': fusion,
                'frames_config': frames,
                'view_bias_config': view_bias,
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='perception_fusion',
            name='perception_fusion',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'cameras_config': cameras,
                'fusion_config': fusion,
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='wrist_tf',
            name='wrist_tf',
            output='screen',
            condition=start_tf,
            parameters=[{
                'config_file': LaunchConfiguration('wrist_tf_config'),
            }],
        ),
        Node(
            package='jcheng_perception_edge',
            executable='ext_tf',
            name='ext_tf',
            output='screen',
            condition=start_tf,
            parameters=[{
                'frames_config': frames,
                'config_file': LaunchConfiguration('ext_tf_config'),
            }],
        ),
    ])
