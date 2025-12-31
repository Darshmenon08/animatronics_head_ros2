#!/usr/bin/env python3
"""
Launch file for Data Collection
Launches:
1. Hardware Interface (bringup)
2. Face Mimic (Passive Mode - extracts features but doesn't drive motors)
3. Slider Control (for manual motor control)
4. Data Collector (records features + motor positions)
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
    camera_id = LaunchConfiguration('camera_id')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Camera device ID'
        ),
        
        # 1. Hardware Interface (Motors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'bringup.launch.py')
            )
        ),
        
        # 2. Face Mimic (Passive Mode)
        Node(
            package='animatronics_head_ros2',
            executable='face_mimic',
            name='face_mimic',
            parameters=[{
                'camera_id': camera_id,
                'show_video': True,
                'drive_motors': False,  # IMPORTANT: Passive mode
            }],
            output='screen'
        ),
        
        # 3. Slider Control (Manual Input)
        Node(
            package='animatronics_head_ros2',
            executable='motor_slider_control',
            name='motor_slider_control',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'motor_limits.yaml')],
        ),
        
        # 4. Data Collector (Recorder)
        Node(
            package='animatronics_head_ros2',
            executable='data_collector',
            name='data_collector',
            output='screen'
        ),
    ])
