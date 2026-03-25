"""Launch only the Gazebo to ROS 2 camera bridge and optional viewer relay."""

from __future__ import annotations

import re
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


GZ_CAMERA_TOPIC_PATTERN = re.compile(
    r'^/world/(?P<world>[^/]+)/model/(?P<model>[^/]+)/'
    r'link/(?P<link>[^/]+)/sensor/(?P<sensor>[^/]+)/'
    r'(?P<kind>image|camera_info)$'
)
TRUE_VALUES = {'1', 'true', 'yes', 'on'}


def _as_bool(value: str) -> bool:
    """Convert a launch argument string into a boolean."""

    return value.strip().lower() in TRUE_VALUES


def _infer_camera_frame(topic_name: str) -> str:
    """Extract the camera frame name from a Gazebo camera topic."""

    match = GZ_CAMERA_TOPIC_PATTERN.match(topic_name)
    if match is None:
        return ''
    return match.group('link')


def _discover_camera_topics(
    model_regex: str,
    attempts: int,
    sleep_sec: float,
) -> dict[str, str]:
    """Poll Gazebo until a matching image/camera_info topic pair exists."""

    model_pattern = re.compile(model_regex)
    last_topics: list[str] = []

    for _ in range(max(attempts, 1)):
        result = subprocess.run(
            ['gz', 'topic', '-l'],
            capture_output=True,
            text=True,
            check=False,
        )

        if result.returncode == 0:
            topics = [line.strip() for line in result.stdout.splitlines()]
            last_topics = [topic for topic in topics if topic]
            grouped_topics: dict[
                tuple[str, str, str, str],
                dict[str, str],
            ] = {}

            for topic_name in last_topics:
                match = GZ_CAMERA_TOPIC_PATTERN.match(topic_name)
                if match is None:
                    continue

                model_name = match.group('model')
                if not model_pattern.search(model_name):
                    continue

                key = (
                    match.group('world'),
                    model_name,
                    match.group('link'),
                    match.group('sensor'),
                )
                grouped_topics.setdefault(key, {})[
                    match.group('kind')
                ] = topic_name

            best_selection: dict[str, str] | None = None
            best_score = -1
            for (world_name, model_name, link_name, sensor_name), kinds in (
                grouped_topics.items()
            ):
                if 'image' not in kinds or 'camera_info' not in kinds:
                    continue

                score = 0
                if model_name.startswith('x500_mono_cam'):
                    score += 100
                if sensor_name == 'camera':
                    score += 20
                if link_name == 'camera_link':
                    score += 10

                if score > best_score:
                    best_score = score
                    best_selection = {
                        'image_topic': kinds['image'],
                        'camera_info_topic': kinds['camera_info'],
                        'camera_frame': link_name,
                        'world_name': world_name,
                        'model_name': model_name,
                    }

            if best_selection is not None:
                return best_selection

        time.sleep(max(sleep_sec, 0.1))

    matching_topics = [
        topic_name
        for topic_name in last_topics
        if GZ_CAMERA_TOPIC_PATTERN.match(topic_name) is not None
    ]
    available = (
        '\n'.join(matching_topics[:20])
        or 'no Gazebo camera topics found'
    )
    raise RuntimeError(
        'Could not auto-discover Gazebo camera topics for model regex '
        f'{model_regex!r}. Available camera-related topics:\n{available}'
    )


def _launch_setup(context, *args, **kwargs):
    """Create bridge and viewer actions after resolving runtime discovery."""

    enable_bridge = _as_bool(
        LaunchConfiguration('enable_bridge').perform(context)
    )
    bridge_clock = _as_bool(
        LaunchConfiguration('bridge_clock').perform(context)
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context)
    )
    start_viewer = _as_bool(
        LaunchConfiguration('start_viewer').perform(context)
    )
    show_viewer_window = _as_bool(
        LaunchConfiguration('show_viewer_window').perform(context)
    )
    log_camera_info = _as_bool(
        LaunchConfiguration('log_camera_info').perform(context)
    )
    enable_aruco_overlay = _as_bool(
        LaunchConfiguration('enable_aruco_overlay').perform(context)
    )
    draw_rejected_candidates = _as_bool(
        LaunchConfiguration('draw_rejected_candidates').perform(context)
    )
    aruco_dictionary = LaunchConfiguration('aruco_dictionary').perform(context)
    model_regex = LaunchConfiguration('model_regex').perform(context)
    attempts = int(
        LaunchConfiguration('topic_discovery_attempts').perform(context)
    )
    sleep_sec = float(
        LaunchConfiguration('topic_discovery_sleep_sec').perform(context)
    )
    manual_image_topic = LaunchConfiguration('image_topic').perform(context)
    manual_camera_info_topic = LaunchConfiguration(
        'camera_info_topic'
    ).perform(context)
    relay_image_topic = LaunchConfiguration('relay_image_topic').perform(context)
    relay_camera_info_topic = LaunchConfiguration(
        'relay_camera_info_topic'
    ).perform(context)

    actions = []
    selection = None

    if enable_bridge or start_viewer:
        if manual_image_topic and manual_camera_info_topic:
            selection = {
                'image_topic': manual_image_topic,
                'camera_info_topic': manual_camera_info_topic,
                'camera_frame': _infer_camera_frame(manual_image_topic),
                'world_name': '',
                'model_name': '',
            }
        else:
            selection = _discover_camera_topics(
                model_regex=model_regex,
                attempts=attempts,
                sleep_sec=sleep_sec,
            )

        actions.append(
            LogInfo(
                msg=(
                    'Resolved Gazebo camera topics: '
                    f"image={selection['image_topic']} "
                    f"camera_info={selection['camera_info_topic']} "
                    f"camera_frame={selection['camera_frame'] or '<pending>'}"
                )
            )
        )

    if enable_bridge:
        bridge_arguments = [
            (
                f"{selection['image_topic']}"
                '@sensor_msgs/msg/Image[gz.msgs.Image'
            ),
            (
                f"{selection['camera_info_topic']}"
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ),
        ]
        if bridge_clock:
            bridge_arguments.append(
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            )

        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_camera_bridge',
                output='screen',
                arguments=bridge_arguments,
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )
    else:
        actions.append(
            LogInfo(
                msg='enable_bridge:=false, ros_gz_bridge will not be started.'
            )
        )

    if start_viewer:
        actions.append(
            Node(
                package='offboard_takeoff',
                executable='camera_viewer',
                name='camera_viewer',
                output='screen',
                parameters=[
                    {
                        'image_topic': selection['image_topic'],
                        'camera_info_topic': selection['camera_info_topic'],
                        'relay_image_topic': relay_image_topic,
                        'relay_camera_info_topic': relay_camera_info_topic,
                        'show_window': show_viewer_window,
                        'log_camera_info': log_camera_info,
                        'enable_aruco_overlay': enable_aruco_overlay,
                        'draw_rejected_candidates': draw_rejected_candidates,
                        'aruco_dictionary': aruco_dictionary,
                        'use_sim_time': use_sim_time,
                    }
                ],
            )
        )
    else:
        actions.append(
            LogInfo(
                msg='start_viewer:=false, camera_viewer relay will not be started.'
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Create the ROS 2 launch description for the camera bridge."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_bridge',
                default_value='true',
                description=(
                    'Start ros_gz_bridge after auto-discovering camera '
                    'topics.'
                ),
            ),
            DeclareLaunchArgument(
                'start_viewer',
                default_value='true',
                description='Start the simple ROS 2 camera viewer and relay.',
            ),
            DeclareLaunchArgument(
                'show_viewer_window',
                default_value='true',
                description='Show the image in an OpenCV window.',
            ),
            DeclareLaunchArgument(
                'log_camera_info',
                default_value='true',
                description='Log the first CameraInfo message for debugging.',
            ),
            DeclareLaunchArgument(
                'enable_aruco_overlay',
                default_value='true',
                description='Draw simple ArUco detections in the viewer window.',
            ),
            DeclareLaunchArgument(
                'aruco_dictionary',
                default_value='DICT_4X4_50',
                description='OpenCV ArUco dictionary name used by the viewer.',
            ),
            DeclareLaunchArgument(
                'draw_rejected_candidates',
                default_value='false',
                description='Also draw rejected ArUco candidates in red.',
            ),
            DeclareLaunchArgument(
                'bridge_clock',
                default_value='true',
                description='Also bridge /clock from Gazebo into ROS 2.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time for the launched nodes.',
            ),
            DeclareLaunchArgument(
                'model_regex',
                default_value='x500_mono_cam(_[0-9]+)?',
                description=(
                    'Regex used to pick the Gazebo camera model instance.'
                ),
            ),
            DeclareLaunchArgument(
                'topic_discovery_attempts',
                default_value='30',
                description=(
                    'How many times to poll gz topic -l before failing.'
                ),
            ),
            DeclareLaunchArgument(
                'topic_discovery_sleep_sec',
                default_value='0.5',
                description='Delay between Gazebo topic discovery attempts.',
            ),
            DeclareLaunchArgument(
                'image_topic',
                default_value='',
                description='Optional manual Gazebo image topic override.',
            ),
            DeclareLaunchArgument(
                'camera_info_topic',
                default_value='',
                description='Optional manual Gazebo CameraInfo topic override.',
            ),
            DeclareLaunchArgument(
                'relay_image_topic',
                default_value='/camera/image_raw',
                description='Stable ROS 2 image topic published by the viewer relay.',
            ),
            DeclareLaunchArgument(
                'relay_camera_info_topic',
                default_value='/camera/camera_info',
                description='Stable ROS 2 CameraInfo topic published by the viewer relay.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
