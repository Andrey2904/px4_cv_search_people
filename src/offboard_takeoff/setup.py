from glob import glob
import os

from setuptools import setup


package_name = 'offboard_takeoff'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dron',
    maintainer_email='dron@example.com',
    description='PX4 offboard mission and ArUco tools for Gazebo SITL',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector = offboard_takeoff.aruco_detector:main',
            'camera_viewer = offboard_takeoff.camera_viewer:main',
            'offboard_takeoff = offboard_takeoff.offboard_takeoff:main',
            'yolo_detector = offboard_takeoff.yolo_detector:main',
        ],
    },
)
