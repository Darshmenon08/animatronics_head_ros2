from setuptools import setup

package_name = 'hardware_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Hardware interface package for animatronics head motor controllers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = hardware_interface.motor_controller:main',
            'dynamixel_controller = hardware_interface.dynamixel_controller:main',
            'eye_controller = hardware_interface.eye_controller:main',
        ],
    },
)
