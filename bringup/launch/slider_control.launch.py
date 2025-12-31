#!/usr/bin/env python3
"""
Launch file for Motor Slider Control GUI
Launches the dynamixel controller and slider control interface
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    
    # Load motor limits config
    motor_limits_config = os.path.join(pkg_share, 'config', 'motor_limits.yaml')
    
    return LaunchDescription([
        # Motor Slider Control GUI
        Node(
            package='bringup',
            executable='motor_slider_control',
            name='motor_slider_control',
            output='screen',
            parameters=[motor_limits_config],
        ),
    ])

