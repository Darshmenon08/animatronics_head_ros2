#!/usr/bin/env python3
"""
Launch file for Complete Face Mimic System
Launches both the hardware interface (bringup) and the face mimic node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('animatronics_head_ros2')
    
    # Launch arguments
    camera_id = LaunchConfiguration('camera_id')
    show_video = LaunchConfiguration('show_video')
    drive_motors = LaunchConfiguration('drive_motors')
    
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
        
        # 1. Hardware Interface (Motors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'bringup.launch.py')
            )
        ),
        
        # 2. Face Mimic Logic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'face_mimic.launch.py')
            ),
            launch_arguments={
                'camera_id': camera_id,
                'show_video': show_video,
                'drive_motors': drive_motors,
            }.items()
        ),
    ])
