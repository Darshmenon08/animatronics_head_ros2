#!/usr/bin/env python3
"""Launch file for Face Mimic node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Camera device ID (0 for /dev/video0, 9 for HP webcam)'
        ),
        DeclareLaunchArgument(
            'show_video',
            default_value='true',
            description='Show video preview window'
        ),
        DeclareLaunchArgument(
            'drive_motors',
            default_value='true',
            description='Whether to drive motors (true) or just publish features (false)'
        ),
        Node(
            package='animatronics_head_ros2',
            executable='face_mimic',
            name='face_mimic',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'show_video': LaunchConfiguration('show_video'),
                'drive_motors': LaunchConfiguration('drive_motors'),
            }],
            output='screen'
        ),
    ])
