"""Launch the RViz scene map publisher for the SAR simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the RViz scene map."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description=(
                    'Use Gazebo simulation time. Keep false unless /clock '
                    'is bridged.'
                ),
            ),
            DeclareLaunchArgument(
                'world_sdf_path',
                default_value=(
                    '/home/dron/PX4-Autopilot/Tools/simulation/gz/'
                    'worlds/forest.sdf'
                ),
                description='Path to the Gazebo world SDF file.',
            ),
            DeclareLaunchArgument(
                'parent_frame_id',
                default_value='map',
                description='Parent RViz fixed frame for the mission map.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                description='Start RViz2 alongside the scene map publisher.',
            ),
            DeclareLaunchArgument(
                'start_top_down_viewer',
                default_value='false',
                description='Start the OpenCV top-down map window.',
            ),
            DeclareLaunchArgument(
                'mission_rotation_deg',
                default_value='90.0',
                description='Rotation applied to PX4 local mission XY.',
            ),
            Node(
                package='offboard_takeoff',
                executable='scene_map_publisher',
                name='scene_map_publisher',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'world_sdf_path': LaunchConfiguration(
                            'world_sdf_path'
                        ),
                        'parent_frame_id': LaunchConfiguration(
                            'parent_frame_id'
                        ),
                    }
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_scene_map',
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
            Node(
                package='offboard_takeoff',
                executable='top_down_map_viewer',
                name='top_down_map_viewer',
                output='screen',
                condition=IfCondition(
                    LaunchConfiguration('start_top_down_viewer')
                ),
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'world_sdf_path': LaunchConfiguration(
                            'world_sdf_path'
                        ),
                        'mission_rotation_deg': ParameterValue(
                            LaunchConfiguration('mission_rotation_deg'),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
