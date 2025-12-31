from setuptools import setup
import os
from glob import glob

package_name = 'animatronics_head_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ROS2 package for animatronics head control with motor limits',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = animatronics_head_ros2.motor_controller:main',
            'dynamixel_controller = animatronics_head_ros2.dynamixel_controller:main',
            'motor_value_checker = animatronics_head_ros2.motor_value_checker:main',
            'eye_controller = animatronics_head_ros2.eye_controller:main',
            'face_mimic = animatronics_head_ros2.face_mimic:main',
            'motor_slider_control = animatronics_head_ros2.motor_slider_control:main',
            'blendshape_mimic = animatronics_head_ros2.blendshape_mimic:main',
            'data_collector = animatronics_head_ros2.data_collector:main',
            'neural_face_mimic = animatronics_head_ros2.neural_face_mimic:main',
            'video_mimic = animatronics_head_ros2.video_mimic:main',
        ],
    },
)
