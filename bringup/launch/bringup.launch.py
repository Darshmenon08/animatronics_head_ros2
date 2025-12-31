import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete animatronics head bringup."""
    
    pkg_share = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_share, 'config', 'motor_limits.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation mode (no hardware)'
        ),
        
        # Motor Controller Node - applies safety limits
        Node(
            package='hardware_interface',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config_file],
            output='screen',
            emulate_tty=True,
        ),
        
        # Dynamixel Controller Node - hardware communication
        Node(
            package='hardware_interface',
            executable='dynamixel_controller',
            name='dynamixel_controller',
            parameters=[config_file],
            output='screen',
            emulate_tty=True,
        ),
    ])

