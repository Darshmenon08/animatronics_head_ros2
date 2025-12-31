import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch eye controller with camera tracking."""
    
    pkg_share = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_share, 'config', 'motor_limits.yaml')
    
    return LaunchDescription([
        # Motor Controller Node
        Node(
            package='hardware_interface',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config_file],
            output='screen',
        ),
        
        # Eye Controller Node
        Node(
            package='hardware_interface',
            executable='eye_controller',
            name='eye_controller',
            parameters=[{'camera_id': 0, 'smoothing_window': 10}],
            output='screen',
        ),
        
        # Dynamixel Controller Node
        Node(
            package='hardware_interface',
            executable='dynamixel_controller',
            name='dynamixel_controller',
            parameters=[config_file],
            output='screen',
        ),
    ])

