"""Publish the measured wrist-camera TF from bus-servo positions.

Run this on the robot together with the edge node.  It needs the vendor
workspace sourced for ``ros_robot_controller_msgs`` and
``/ros_robot_controller/bus_servo/get_state``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    default_config = os.path.join(
        get_package_share_directory('jcheng_perception_edge'),
        'config', 'wrist_tf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config),
        Node(
            package='jcheng_perception_edge',
            executable='wrist_tf',
            name='wrist_tf',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),
    ])
