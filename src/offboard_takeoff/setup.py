from setuptools import setup
import os
from glob import glob

package_name = 'offboard_takeoff'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dron',
    maintainer_email='dron@example.com',
    description='Minimal PX4 offboard takeoff node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_viewer = offboard_takeoff.camera_viewer:main',
            'offboard_takeoff = offboard_takeoff.offboard_takeoff:main',
        ],
    },
)
