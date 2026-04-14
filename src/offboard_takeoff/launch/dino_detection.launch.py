"""Launch Grounding DINO person detection on the relayed drone camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the camera bridge, viewer, and Grounding DINO detector."""

    shared_launch = PythonLaunchDescriptionSource(
        [
            FindPackageShare('offboard_takeoff'),
            '/launch/person_detection.launch.py',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model_id',
                default_value='IDEA-Research/grounding-dino-tiny',
                description='Hugging Face model ID or local directory.',
            ),
            DeclareLaunchArgument(
                'cache_dir',
                default_value='/home/dron/px4_offboard_clean_ws/models',
                description='Grounding DINO cache directory.',
            ),
            DeclareLaunchArgument(
                'prompt_text',
                default_value='person',
                description='Grounding DINO prompt text.',
            ),
            DeclareLaunchArgument(
                'box_threshold',
                default_value='0.35',
                description='Minimum box confidence threshold.',
            ),
            DeclareLaunchArgument(
                'text_threshold',
                default_value='0.25',
                description='Minimum text matching threshold.',
            ),
            DeclareLaunchArgument(
                'processing_max_rate_hz',
                default_value='2.0',
                description='Maximum Grounding DINO inference rate.',
            ),
            IncludeLaunchDescription(
                shared_launch,
                launch_arguments={
                    'start_yolo': 'false',
                }.items(),
            ),
            Node(
                package='offboard_takeoff',
                executable='grounding_dino_detector',
                name='grounding_dino_detector',
                output='screen',
                parameters=[
                    {
                        'image_topic': '/camera/image_raw',
                        'model_id': LaunchConfiguration('model_id'),
                        'cache_dir': LaunchConfiguration('cache_dir'),
                        'prompt_text': LaunchConfiguration('prompt_text'),
                        'target_labels': ['person'],
                        'box_threshold': LaunchConfiguration(
                            'box_threshold'
                        ),
                        'text_threshold': LaunchConfiguration(
                            'text_threshold'
                        ),
                        'processing_max_rate_hz': LaunchConfiguration(
                            'processing_max_rate_hz'
                        ),
                        'publish_debug_image': True,
                        'show_debug_window': True,
                        'use_half': False,
                        'use_sim_time': True,
                    }
                ],
            ),
        ]
    )
