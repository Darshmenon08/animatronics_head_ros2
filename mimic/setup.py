from setuptools import setup

package_name = 'mimic'

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
    description='Face mimicry package for animatronics head',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_mimic = mimic.face_mimic:main',
            'blendshape_mimic = mimic.blendshape_mimic:main',
            'neural_face_mimic = mimic.neural_face_mimic:main',
            'video_mimic = mimic.video_mimic:main',
        ],
    },
)
