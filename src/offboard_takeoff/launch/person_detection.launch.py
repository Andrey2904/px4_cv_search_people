"""Preferred launch entry point for camera bridge and person detection."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Forward to the shared person-search launch configuration."""

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare('offboard_takeoff'),
                        '/launch/aruco_detection.launch.py',
                    ]
                )
            )
        ]
    )
