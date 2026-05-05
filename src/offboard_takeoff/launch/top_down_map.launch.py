"""Launch the OpenCV top-down Gazebo world map viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the top-down map viewer."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use Gazebo simulation time.',
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
                'window_width',
                default_value='1100',
                description='OpenCV map window width in pixels.',
            ),
            DeclareLaunchArgument(
                'window_height',
                default_value='850',
                description='OpenCV map window height in pixels.',
            ),
            DeclareLaunchArgument(
                'map_padding_m',
                default_value='5.0',
                description='Padding around extreme objects in meters.',
            ),
            DeclareLaunchArgument(
                'draw_object_labels',
                default_value='true',
                description='Draw object names on the top-down map.',
            ),
            DeclareLaunchArgument(
                'world_rotation_deg',
                default_value='0.0',
                description='Rotation applied to Gazebo SDF object XY.',
            ),
            DeclareLaunchArgument(
                'mission_rotation_deg',
                default_value='90.0',
                description='Rotation applied to PX4 local mission XY.',
            ),
            DeclareLaunchArgument(
                'mission_swap_xy',
                default_value='false',
                description='Swap PX4 local mission X and Y before rotation.',
            ),
            DeclareLaunchArgument(
                'mission_invert_x',
                default_value='false',
                description='Mirror PX4 local mission X before rotation.',
            ),
            DeclareLaunchArgument(
                'mission_invert_y',
                default_value='true',
                description='Mirror PX4 local mission Y before rotation.',
            ),
            DeclareLaunchArgument(
                'mission_offset_x',
                default_value='0.0',
                description='X offset applied to PX4 local mission XY.',
            ),
            DeclareLaunchArgument(
                'mission_offset_y',
                default_value='0.0',
                description='Y offset applied to PX4 local mission XY.',
            ),
            Node(
                package='offboard_takeoff',
                executable='top_down_map_viewer',
                name='top_down_map_viewer',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'world_sdf_path': LaunchConfiguration(
                            'world_sdf_path'
                        ),
                        'window_width': LaunchConfiguration('window_width'),
                        'window_height': LaunchConfiguration(
                            'window_height'
                        ),
                        'map_padding_m': ParameterValue(
                            LaunchConfiguration('map_padding_m'),
                            value_type=float,
                        ),
                        'draw_object_labels': LaunchConfiguration(
                            'draw_object_labels'
                        ),
                        'world_rotation_deg': ParameterValue(
                            LaunchConfiguration('world_rotation_deg'),
                            value_type=float,
                        ),
                        'mission_rotation_deg': ParameterValue(
                            LaunchConfiguration('mission_rotation_deg'),
                            value_type=float,
                        ),
                        'mission_swap_xy': ParameterValue(
                            LaunchConfiguration('mission_swap_xy'),
                            value_type=bool,
                        ),
                        'mission_invert_x': ParameterValue(
                            LaunchConfiguration('mission_invert_x'),
                            value_type=bool,
                        ),
                        'mission_invert_y': ParameterValue(
                            LaunchConfiguration('mission_invert_y'),
                            value_type=bool,
                        ),
                        'mission_offset_x': ParameterValue(
                            LaunchConfiguration('mission_offset_x'),
                            value_type=float,
                        ),
                        'mission_offset_y': ParameterValue(
                            LaunchConfiguration('mission_offset_y'),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
