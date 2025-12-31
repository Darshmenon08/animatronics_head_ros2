#!/usr/bin/env python3
"""
Launch file for Video Mimic durability testing.
Plays back recorded expressions on loop for long-duration testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('animatronics_head_ros2')
    
    # Launch arguments
    video_path = LaunchConfiguration('video_path')
    loop = LaunchConfiguration('loop')
    duration_mins = LaunchConfiguration('duration_mins')
    show_video = LaunchConfiguration('show_video')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            default_value='',
            description='Path to video file (empty = use latest recording)'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Loop the video'
        ),
        DeclareLaunchArgument(
            'duration_mins',
            default_value='30',
            description='Approximate duration in minutes (0 = infinite)'
        ),
        DeclareLaunchArgument(
            'show_video',
            default_value='true',
            description='Show video preview'
        ),
        
        # 1. Hardware Interface (Motors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'bringup.launch.py')
            )
        ),
        
        # 2. Video Mimic Node
        Node(
            package='animatronics_head_ros2',
            executable='video_mimic',
            name='video_mimic',
            parameters=[{
                'video_path': video_path,
                'loop': loop,
                'loop_count': 0,  # Infinite loops, duration controlled by user
                'show_video': show_video,
            }],
            output='screen'
        ),
    ])
