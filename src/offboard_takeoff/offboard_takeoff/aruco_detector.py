"""ROS 2 ArUco detector for the PX4 Gazebo monocular camera."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Iterable

import cv2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml


ARUCO_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith('DICT_')
}

IMAGE_TOPIC_TYPE = 'sensor_msgs/msg/Image'
CAMERA_INFO_TOPIC_TYPE = 'sensor_msgs/msg/CameraInfo'
MODEL_TOPIC_PATTERN = re.compile(
    r'^/world/(?P<world>[^/]+)/model/(?P<model>[^/]+)/'
    r'link/(?P<link>[^/]+)/sensor/(?P<sensor>[^/]+)/image$'
)


@dataclass(frozen=True)
class TopicSelection:
    """Resolved camera topics and metadata for the detector."""

    image_topic: str
    camera_info_topic: str
    camera_frame: str
    model_name: str
    world_name: str


@dataclass(frozen=True)
class MarkerDetection:
    """Single ArUco pose estimate in the camera frame."""

    marker_id: int
    pose: Pose
    rvec: np.ndarray
    tvec: np.ndarray


def marker_object_points(marker_size_m: float) -> np.ndarray:
    """Return square marker corners in the marker coordinate frame."""

    half_size = marker_size_m / 2.0
    return np.array(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float32,
    )


def quaternion_from_matrix(rotation_matrix: np.ndarray) -> tuple[float, ...]:
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""

    trace = float(np.trace(rotation_matrix))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
        return qx, qy, qz, qw

    diagonal = np.diag(rotation_matrix)
    max_index = int(np.argmax(diagonal))

    if max_index == 0:
        scale = math.sqrt(
            1.0
            + rotation_matrix[0, 0]
            - rotation_matrix[1, 1]
            - rotation_matrix[2, 2]
        ) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
        return qx, qy, qz, qw

    if max_index == 1:
        scale = math.sqrt(
            1.0
            + rotation_matrix[1, 1]
            - rotation_matrix[0, 0]
            - rotation_matrix[2, 2]
        ) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
        return qx, qy, qz, qw

    scale = math.sqrt(
        1.0
        + rotation_matrix[2, 2]
        - rotation_matrix[0, 0]
        - rotation_matrix[1, 1]
    ) * 2.0
    qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
    qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
    qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
    qz = 0.25 * scale
    return qx, qy, qz, qw


def pose_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    """Convert solvePnP output into a ROS pose."""

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    qx, qy, qz, qw = quaternion_from_matrix(rotation_matrix)

    pose = Pose()
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def transform_to_matrix(translation, rotation) -> np.ndarray:
    """Build a 4x4 transform matrix from translation and quaternion."""

    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (yy + zz)
    matrix[0, 1] = 2.0 * (xy - wz)
    matrix[0, 2] = 2.0 * (xz + wy)
    matrix[1, 0] = 2.0 * (xy + wz)
    matrix[1, 1] = 1.0 - 2.0 * (xx + zz)
    matrix[1, 2] = 2.0 * (yz - wx)
    matrix[2, 0] = 2.0 * (xz - wy)
    matrix[2, 1] = 2.0 * (yz + wx)
    matrix[2, 2] = 1.0 - 2.0 * (xx + yy)
    matrix[0, 3] = float(translation.x)
    matrix[1, 3] = float(translation.y)
    matrix[2, 3] = float(translation.z)
    return matrix


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Convert a pose message into a 4x4 transform matrix."""

    return transform_to_matrix(pose.position, pose.orientation)


def pose_from_matrix(matrix: np.ndarray) -> Pose:
    """Convert a 4x4 homogeneous transform into a ROS pose."""

    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class ArucoDetector(Node):
    """Detect ArUco markers from the PX4 Gazebo monocular camera."""

    def __init__(self):
        super().__init__('aruco_detector')

        self._declare_parameters()
        self._load_parameters()

        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.topic_selection: TopicSelection | None = None
        self.image_subscription = None
        self.camera_info_subscription = None
        self.object_points = marker_object_points(self.marker_size_m)
        self.current_camera_frame = self.camera_frame
        self.last_processed_frame_ns = 0
        self.last_debug_image_pub_ns = 0
        self.last_detected_marker_ids: tuple[int, ...] = ()
        self.last_seen_marker_ids: tuple[int, ...] = ()
        self.last_seen_marker_ns = 0
        self.model_name = ''
        self.world_name = ''
        self.base_frame: str | None = None
        self.world_frame: str | None = None

        self.aruco_dictionary = self._create_dictionary(
            self.aruco_dictionary_name
        )
        self.detector_parameters = self._create_detector_parameters()
        self.aruco_detector = self._create_aruco_detector()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            'aruco/debug_image',
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_detected_pub = self.create_publisher(
            Bool,
            'aruco/marker_detected',
            status_qos,
        )
        self.detection_count_pub = self.create_publisher(
            Int32,
            'aruco/detection_count',
            status_qos,
        )
        self.marker_ids_pub = self.create_publisher(
            Int32MultiArray,
            'aruco/marker_ids',
            status_qos,
        )
        self.camera_pose_pub = self.create_publisher(
            PoseArray,
            'aruco/poses/camera',
            10,
        )
        self.base_pose_pub = self.create_publisher(
            PoseArray,
            'aruco/poses/base_link',
            10,
        )
        self.world_pose_pub = self.create_publisher(
            PoseArray,
            'aruco/poses/world',
            10,
        )
        self.camera_marker_pub = self.create_publisher(
            MarkerArray,
            'aruco/markers/camera',
            10,
        )
        self.base_marker_pub = self.create_publisher(
            MarkerArray,
            'aruco/markers/base_link',
            10,
        )
        self.world_marker_pub = self.create_publisher(
            MarkerArray,
            'aruco/markers/world',
            10,
        )

        self.discovery_timer = self.create_timer(
            self.topic_discovery_period_sec,
            self._discovery_timer_callback,
        )
        self.frame_discovery_timer = self.create_timer(
            self.frame_discovery_period_sec,
            self._frame_discovery_timer_callback,
        )

        self.get_logger().info(
            'ArucoDetector started, waiting for camera topics and CameraInfo'
        )

        if self.image_topic and self.camera_info_topic:
            selection = TopicSelection(
                image_topic=self.image_topic,
                camera_info_topic=self.camera_info_topic,
                camera_frame=self.camera_frame,
                model_name=self._model_name_from_topic(self.image_topic),
                world_name=self._world_name_from_topic(self.image_topic),
            )
            self._configure_camera_subscriptions(selection)

    def _declare_parameters(self):
        """Declare detector configuration parameters."""

        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('marker_size_m', 0.2)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter(
            'discovery_topic_regex',
            r'(x500_mono_cam|mono_cam|camera)',
        )
        self.declare_parameter('topic_discovery_period_sec', 1.0)
        self.declare_parameter('frame_discovery_period_sec', 2.0)
        self.declare_parameter('processing_max_rate_hz', 8.0)
        self.declare_parameter('detection_hold_sec', 1.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_max_rate_hz', 5.0)
        self.declare_parameter('debug_image_scale', 0.5)
        self.declare_parameter('log_detection_events', True)
        self.declare_parameter('draw_rejected_candidates', False)
        self.declare_parameter('corner_subpix_refinement', True)
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('base_frame_hints', ['base_link'])
        self.declare_parameter('world_frame_hints', ['world', 'map', 'odom'])

    def _load_parameters(self):
        """Load detector configuration from the ROS parameter server."""

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.aruco_dictionary_name = str(
            self.get_parameter('aruco_dictionary').value
        )
        self.discovery_topic_regex = str(
            self.get_parameter('discovery_topic_regex').value
        )
        self.topic_discovery_period_sec = float(
            self.get_parameter('topic_discovery_period_sec').value
        )
        self.frame_discovery_period_sec = float(
            self.get_parameter('frame_discovery_period_sec').value
        )
        self.processing_max_rate_hz = float(
            self.get_parameter('processing_max_rate_hz').value
        )
        self.detection_hold_sec = float(
            self.get_parameter('detection_hold_sec').value
        )
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )
        self.debug_image_max_rate_hz = float(
            self.get_parameter('debug_image_max_rate_hz').value
        )
        self.debug_image_scale = float(
            self.get_parameter('debug_image_scale').value
        )
        self.log_detection_events = bool(
            self.get_parameter('log_detection_events').value
        )
        self.draw_rejected_candidates = bool(
            self.get_parameter('draw_rejected_candidates').value
        )
        self.corner_subpix_refinement = bool(
            self.get_parameter('corner_subpix_refinement').value
        )
        self.tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self.base_frame_hints = tuple(
            str(value)
            for value in self.get_parameter('base_frame_hints').value
        )
        self.world_frame_hints = tuple(
            str(value)
            for value in self.get_parameter('world_frame_hints').value
        )

    def _create_dictionary(self, dictionary_name: str):
        """Create the configured OpenCV ArUco dictionary."""

        if dictionary_name not in ARUCO_DICTIONARIES:
            supported = ', '.join(sorted(ARUCO_DICTIONARIES))
            raise ValueError(
                f'Unsupported ArUco dictionary "{dictionary_name}". '
                f'Available dictionaries: {supported}'
            )

        dictionary_id = ARUCO_DICTIONARIES[dictionary_name]
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _create_detector_parameters(self):
        """Build OpenCV ArUco detector parameters."""

        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _create_aruco_detector(self):
        """Instantiate the modern ArUco detector API when available."""

        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(
                self.aruco_dictionary,
                self.detector_parameters,
            )
        return None

    def _discovery_timer_callback(self):
        """Keep trying to resolve the camera topics until subscribed."""

        if self.image_subscription is not None:
            return

        selection = self._select_camera_topics()
        if selection is None:
            return

        self._configure_camera_subscriptions(selection)

    def _frame_discovery_timer_callback(self):
        """Refresh optional TF target frames when they become available."""

        self._discover_target_frames()

    def _select_camera_topics(self) -> TopicSelection | None:
        """Find the most likely bridged PX4 Gazebo camera topic pair."""

        topic_names_and_types = self.get_topic_names_and_types()

        image_topics = [
            topic_name
            for topic_name, topic_types in topic_names_and_types
            if IMAGE_TOPIC_TYPE in topic_types
            and topic_name not in {'/aruco/debug_image', 'aruco/debug_image'}
        ]
        camera_info_topics = {
            topic_name
            for topic_name, topic_types in topic_names_and_types
            if CAMERA_INFO_TOPIC_TYPE in topic_types
        }

        if self.image_topic and self.camera_info_topic:
            if (
                self.image_topic in image_topics
                and self.camera_info_topic in camera_info_topics
            ):
                return TopicSelection(
                    image_topic=self.image_topic,
                    camera_info_topic=self.camera_info_topic,
                    camera_frame=self.camera_frame,
                    model_name=self._model_name_from_topic(self.image_topic),
                    world_name=self._world_name_from_topic(self.image_topic),
                )

        best_score = -1
        best_selection: TopicSelection | None = None

        for image_topic in image_topics:
            camera_info_topic = self._matching_camera_info_topic(
                image_topic,
                camera_info_topics,
            )
            if camera_info_topic is None:
                continue

            score = self._topic_score(image_topic)
            if score <= best_score:
                continue

            best_score = score
            best_selection = TopicSelection(
                image_topic=image_topic,
                camera_info_topic=camera_info_topic,
                camera_frame=self._infer_camera_frame(image_topic),
                model_name=self._model_name_from_topic(image_topic),
                world_name=self._world_name_from_topic(image_topic),
            )

        return best_selection

    def _matching_camera_info_topic(
        self,
        image_topic: str,
        camera_info_topics: Iterable[str],
    ) -> str | None:
        """Find the matching CameraInfo topic for an image topic."""

        if image_topic.endswith('/image'):
            candidate = f'{image_topic[:-len("/image")]}/camera_info'
            if candidate in camera_info_topics:
                return candidate

        if image_topic.endswith('/image_raw'):
            candidate = f'{image_topic[:-len("/image_raw")]}/camera_info'
            if candidate in camera_info_topics:
                return candidate

        for topic_name in camera_info_topics:
            if topic_name.rsplit('/', 1)[0] == image_topic.rsplit('/', 1)[0]:
                return topic_name

        return None

    def _topic_score(self, topic_name: str) -> int:
        """Rank candidate image topics for the detector."""

        score = 0
        if re.search(self.discovery_topic_regex, topic_name):
            score += 50
        if '/sensor/' in topic_name and '/camera/' in topic_name:
            score += 20
        if topic_name.endswith('/image') or topic_name.endswith('/image_raw'):
            score += 20
        if '/world/' in topic_name and '/model/' in topic_name:
            score += 10
        if 'x500_mono_cam' in topic_name:
            score += 100
        return score

    def _configure_camera_subscriptions(self, selection: TopicSelection):
        """Subscribe to the resolved image and CameraInfo topics."""

        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
        if self.camera_info_subscription is not None:
            self.destroy_subscription(self.camera_info_subscription)

        self.topic_selection = selection
        self.model_name = selection.model_name
        self.world_name = selection.world_name

        if selection.camera_frame:
            self.current_camera_frame = selection.camera_frame

        self.image_subscription = self.create_subscription(
            Image,
            selection.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            selection.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Resolved camera topics: '
            f'image={selection.image_topic}, '
            f'camera_info={selection.camera_info_topic}, '
            f'camera_frame={selection.camera_frame or "<pending header>"}'
        )

    def _camera_info_callback(self, msg: CameraInfo):
        """Cache the latest camera intrinsics and frame name."""

        if not any(msg.k):
            self.get_logger().warn(
                'Received CameraInfo without intrinsic matrix, '
                'pose estimation is not possible yet'
            )
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        if msg.header.frame_id:
            if msg.header.frame_id != self.current_camera_frame:
                self.get_logger().info(
                    f'Camera frame resolved from CameraInfo: '
                    f'{msg.header.frame_id}'
                )
            self.current_camera_frame = msg.header.frame_id

        self._discover_target_frames()

    def _image_callback(self, msg: Image):
        """Run ArUco detection and publish poses plus debug image."""

        if self.camera_matrix is None or self.dist_coeffs is None:
            self._warn_throttled(
                'Waiting for CameraInfo before processing images',
                warning_key='camera_info',
            )
            return

        if not self._should_process_frame():
            return

        publish_debug_image = self._should_publish_debug_image()

        frame = self._image_to_bgr(msg)
        if frame is None:
            return

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect_markers(gray_frame)
        debug_frame = frame.copy() if publish_debug_image else None
        detections: list[MarkerDetection] = []

        if ids is not None and len(ids) > 0:
            if debug_frame is not None:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            for marker_corners, marker_id_array in zip(corners, ids):
                marker_id = int(marker_id_array[0])
                refined_corners = self._refine_corners(
                    gray_frame,
                    marker_corners,
                )
                detection = self._estimate_marker_pose(
                    marker_id,
                    refined_corners,
                )
                if detection is None:
                    continue

                detections.append(detection)

                if debug_frame is None:
                    continue

                cv2.drawFrameAxes(
                    debug_frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    detection.rvec,
                    detection.tvec,
                    self.marker_size_m * 0.5,
                    2,
                )

                center = np.mean(
                    refined_corners.reshape(4, 2),
                    axis=0,
                ).astype(int)
                cv2.putText(
                    debug_frame,
                    f'id={marker_id}',
                    (int(center[0] + 10), int(center[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        if (
            self.draw_rejected_candidates
            and rejected
            and debug_frame is not None
        ):
            cv2.aruco.drawDetectedMarkers(
                debug_frame,
                rejected,
                borderColor=(0, 0, 255),
            )

        detected_marker_ids = tuple(
            sorted(detection.marker_id for detection in detections)
        )
        self._publish_detection_status(detected_marker_ids)

        camera_frame = self._resolve_camera_frame(msg)
        self._publish_pose_set(
            detections=detections,
            frame_name=camera_frame,
            header=msg,
            pose_publisher=self.camera_pose_pub,
            marker_publisher=self.camera_marker_pub,
            marker_namespace='camera',
        )

        self._publish_transformed_pose_set(
            detections=detections,
            source_frame=camera_frame,
            header=msg,
            target_frame=self.base_frame,
            pose_publisher=self.base_pose_pub,
            marker_publisher=self.base_marker_pub,
            marker_namespace='base_link',
        )
        self._publish_transformed_pose_set(
            detections=detections,
            source_frame=camera_frame,
            header=msg,
            target_frame=self.world_frame,
            pose_publisher=self.world_pose_pub,
            marker_publisher=self.world_marker_pub,
            marker_namespace='world',
        )

        if debug_frame is not None:
            debug_frame = self._prepare_debug_frame_for_publish(debug_frame)
            debug_msg = self._bgr_to_image_message(
                frame=debug_frame,
                stamp=msg.header.stamp,
                frame_id=camera_frame,
            )
            self.debug_image_pub.publish(debug_msg)

    def _publish_detection_status(
        self,
        detected_marker_ids: tuple[int, ...],
    ):
        """Publish lightweight marker status for terminal-based checks."""

        now_ns = self.get_clock().now().nanoseconds
        if detected_marker_ids:
            self.last_seen_marker_ids = detected_marker_ids
            self.last_seen_marker_ns = now_ns

        effective_marker_ids: tuple[int, ...] = ()
        hold_period_ns = int(max(self.detection_hold_sec, 0.0) * 1e9)
        if detected_marker_ids:
            effective_marker_ids = detected_marker_ids
        elif (
            hold_period_ns > 0
            and self.last_seen_marker_ids
            and now_ns - self.last_seen_marker_ns <= hold_period_ns
        ):
            effective_marker_ids = self.last_seen_marker_ids

        self.marker_detected_pub.publish(
            Bool(data=bool(effective_marker_ids))
        )
        self.detection_count_pub.publish(
            Int32(data=len(effective_marker_ids))
        )
        self.marker_ids_pub.publish(
            Int32MultiArray(data=list(effective_marker_ids))
        )

        if not self.log_detection_events:
            self.last_detected_marker_ids = effective_marker_ids
            return

        if effective_marker_ids == self.last_detected_marker_ids:
            return

        if effective_marker_ids:
            self.get_logger().info(
                f'Detected ArUco markers: {list(effective_marker_ids)}'
            )
        elif self.last_detected_marker_ids:
            self.get_logger().info('Lost sight of all ArUco markers')

        self.last_detected_marker_ids = effective_marker_ids

    def _should_process_frame(self) -> bool:
        """Throttle expensive CV work to keep the simulator responsive."""

        if self.processing_max_rate_hz <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(1e9 / self.processing_max_rate_hz)
        if now_ns - self.last_processed_frame_ns < min_period_ns:
            return False

        self.last_processed_frame_ns = now_ns
        return True

    def _should_publish_debug_image(self) -> bool:
        """Throttle debug image publishing to keep viewers responsive."""

        if not self.publish_debug_image:
            return False

        if self.debug_image_max_rate_hz <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(1e9 / self.debug_image_max_rate_hz)
        if now_ns - self.last_debug_image_pub_ns < min_period_ns:
            return False

        self.last_debug_image_pub_ns = now_ns
        return True

    def _prepare_debug_frame_for_publish(
        self,
        frame: np.ndarray,
    ) -> np.ndarray:
        """Optionally downscale the debug frame before publishing it."""

        if self.debug_image_scale <= 0.0 or self.debug_image_scale == 1.0:
            return frame

        width = max(int(frame.shape[1] * self.debug_image_scale), 1)
        height = max(int(frame.shape[0] * self.debug_image_scale), 1)
        return cv2.resize(
            frame,
            (width, height),
            interpolation=cv2.INTER_AREA,
        )

    def _detect_markers(self, gray_frame: np.ndarray):
        """Run OpenCV ArUco detection using the available API."""

        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray_frame)

        return cv2.aruco.detectMarkers(
            gray_frame,
            self.aruco_dictionary,
            parameters=self.detector_parameters,
        )

    def _refine_corners(
        self,
        gray_frame: np.ndarray,
        marker_corners: np.ndarray,
    ) -> np.ndarray:
        """Optionally refine corners to improve solvePnP stability."""

        if not self.corner_subpix_refinement:
            return marker_corners

        refined_corners = marker_corners.reshape(4, 1, 2).astype(np.float32)
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.01,
        )
        cv2.cornerSubPix(
            gray_frame,
            refined_corners,
            winSize=(5, 5),
            zeroZone=(-1, -1),
            criteria=criteria,
        )
        return refined_corners.reshape(1, 4, 2)

    def _estimate_marker_pose(
        self,
        marker_id: int,
        marker_corners: np.ndarray,
    ) -> MarkerDetection | None:
        """Estimate a marker pose using solvePnP."""

        image_points = marker_corners.reshape(4, 2).astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            self.get_logger().warn(f'solvePnP failed for marker {marker_id}')
            return None

        pose = pose_from_rvec_tvec(rvec, tvec.reshape(3))
        return MarkerDetection(
            marker_id=marker_id,
            pose=pose,
            rvec=rvec,
            tvec=tvec,
        )

    def _resolve_camera_frame(self, msg: Image) -> str:
        """Pick the best currently known camera frame name."""

        if msg.header.frame_id:
            self.current_camera_frame = msg.header.frame_id
        elif (
            not self.current_camera_frame
            and self.topic_selection is not None
        ):
            self.current_camera_frame = self.topic_selection.camera_frame

        return self.current_camera_frame or 'camera_link'

    def _publish_pose_set(
        self,
        detections: list[MarkerDetection],
        frame_name: str,
        header: Image,
        pose_publisher,
        marker_publisher,
        marker_namespace: str,
    ):
        """Publish marker poses and visualization markers in one frame."""

        pose_array = PoseArray()
        pose_array.header = header.header
        pose_array.header.frame_id = frame_name
        pose_array.poses = [detection.pose for detection in detections]
        pose_publisher.publish(pose_array)

        marker_publisher.publish(
            self._build_marker_array(
                detections=detections,
                frame_name=frame_name,
                header=header,
                marker_namespace=marker_namespace,
            )
        )

    def _publish_transformed_pose_set(
        self,
        detections: list[MarkerDetection],
        source_frame: str,
        header: Image,
        target_frame: str | None,
        pose_publisher,
        marker_publisher,
        marker_namespace: str,
    ):
        """Publish the marker set transformed into another TF frame."""

        if not target_frame:
            pose_array = PoseArray()
            pose_array.header = header.header
            pose_array.header.frame_id = ''
            pose_publisher.publish(pose_array)

            delete_all = MarkerArray()
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            delete_all.markers.append(clear_marker)
            marker_publisher.publish(delete_all)
            return

        transformed_detections: list[MarkerDetection] = []
        for detection in detections:
            transformed_pose = self._transform_pose(
                pose=detection.pose,
                source_frame=source_frame,
                target_frame=target_frame,
                stamp=header.header.stamp,
            )
            if transformed_pose is None:
                return

            transformed_detections.append(
                MarkerDetection(
                    marker_id=detection.marker_id,
                    pose=transformed_pose,
                    rvec=detection.rvec,
                    tvec=detection.tvec,
                )
            )

        self._publish_pose_set(
            detections=transformed_detections,
            frame_name=target_frame,
            header=header,
            pose_publisher=pose_publisher,
            marker_publisher=marker_publisher,
            marker_namespace=marker_namespace,
        )

    def _transform_pose(
        self,
        pose: Pose,
        source_frame: str,
        target_frame: str,
        stamp,
    ) -> Pose | None:
        """Transform a pose using the tf2 buffer if the transform exists."""

        transform_time = Time.from_msg(stamp)
        timeout = Duration(seconds=self.tf_lookup_timeout_sec)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                transform_time,
                timeout=timeout,
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=timeout,
                )
            except TransformException as error:
                self._warn_throttled(
                    f'No TF from {source_frame} to {target_frame}: {error}',
                    warning_key=f'tf:{source_frame}->{target_frame}',
                )
                return None

        target_from_source = transform_to_matrix(
            transform.transform.translation,
            transform.transform.rotation,
        )
        source_from_marker = pose_to_matrix(pose)
        return pose_from_matrix(target_from_source @ source_from_marker)

    def _build_marker_array(
        self,
        detections: list[MarkerDetection],
        frame_name: str,
        header: Image,
        marker_namespace: str,
    ) -> MarkerArray:
        """Create an RViz-friendly MarkerArray for all detections."""

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        stamp = header.header.stamp
        thickness = max(self.marker_size_m * 0.02, 0.005)
        text_size = max(self.marker_size_m * 0.4, 0.1)

        for detection in detections:
            board_marker = Marker()
            board_marker.header.frame_id = frame_name
            board_marker.header.stamp = stamp
            board_marker.ns = marker_namespace
            board_marker.id = detection.marker_id * 2
            board_marker.type = Marker.CUBE
            board_marker.action = Marker.ADD
            board_marker.pose = detection.pose
            board_marker.scale.x = self.marker_size_m
            board_marker.scale.y = self.marker_size_m
            board_marker.scale.z = thickness
            board_marker.color.r = 0.15
            board_marker.color.g = 0.9
            board_marker.color.b = 0.25
            board_marker.color.a = 0.7
            marker_array.markers.append(board_marker)

            text_marker = Marker()
            text_marker.header.frame_id = frame_name
            text_marker.header.stamp = stamp
            text_marker.ns = f'{marker_namespace}_labels'
            text_marker.id = detection.marker_id * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = detection.pose
            text_marker.pose.position.z += self.marker_size_m * 0.75
            text_marker.scale.z = text_size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'id={detection.marker_id}'
            marker_array.markers.append(text_marker)

        return marker_array

    def _discover_target_frames(self):
        """Auto-select connected base/world TF frames when available."""

        camera_frame = self.current_camera_frame
        if not camera_frame:
            return

        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return

        if not frames_yaml:
            return

        frame_tree = yaml.safe_load(frames_yaml) or {}
        frame_names = tuple(frame_tree.keys())
        if not frame_names:
            return

        base_candidate = self._best_frame_candidate(
            frame_names=frame_names,
            preferred_names=self.base_frame_hints,
            role='base',
        )
        if (
            base_candidate is not None
            and self._can_transform(base_candidate, camera_frame)
            and base_candidate != self.base_frame
        ):
            self.base_frame = base_candidate
            self.get_logger().info(
                f'Connected base frame discovered: {self.base_frame}'
            )

        world_candidate = self._best_frame_candidate(
            frame_names=frame_names,
            preferred_names=self.world_frame_hints,
            role='world',
        )
        if (
            world_candidate is not None
            and self._can_transform(world_candidate, camera_frame)
            and world_candidate != self.world_frame
        ):
            self.world_frame = world_candidate
            self.get_logger().info(
                f'Connected world frame discovered: {self.world_frame}'
            )

    def _best_frame_candidate(
        self,
        frame_names: Iterable[str],
        preferred_names: Iterable[str],
        role: str,
    ) -> str | None:
        """Score TF frames to find the best base/world target frame."""

        best_score = -1
        best_frame: str | None = None
        preferred_names = tuple(preferred_names)

        for frame_name in frame_names:
            score = 0

            if frame_name == self.current_camera_frame:
                continue

            if frame_name in preferred_names:
                score += 100

            if role == 'base':
                if frame_name == 'base_link':
                    score += 90
                if frame_name.endswith('/base_link'):
                    score += 80
                if frame_name.endswith('/base_footprint'):
                    score += 70
            else:
                if frame_name == 'world':
                    score += 90
                if frame_name == 'map':
                    score += 80
                if frame_name == 'odom':
                    score += 70
                if frame_name.endswith('/world'):
                    score += 60

            if self.model_name and self.model_name in frame_name:
                score += 40

            if score > best_score:
                best_score = score
                best_frame = frame_name

        return best_frame

    def _can_transform(self, target_frame: str, source_frame: str) -> bool:
        """Return whether TF knows how to transform between two frames."""

        return self.tf_buffer.can_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        )

    def _bgr_to_image_message(
        self,
        frame: np.ndarray,
        stamp,
        frame_id: str,
    ) -> Image:
        """Convert a BGR OpenCV frame into a ROS Image message."""

        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id

        contiguous_frame = np.ascontiguousarray(frame)
        image_msg.height = int(contiguous_frame.shape[0])
        image_msg.width = int(contiguous_frame.shape[1])
        image_msg.is_bigendian = False

        if contiguous_frame.ndim == 2:
            image_msg.encoding = 'mono8'
        else:
            image_msg.encoding = 'bgr8'

        image_msg.step = int(contiguous_frame.strides[0])
        image_msg.data = contiguous_frame.tobytes()
        return image_msg

    def _ros_image_to_numpy(self, msg: Image) -> np.ndarray:
        """Convert a ROS Image message into a contiguous numpy array."""

        channel_map = {
            'mono8': 1,
            '8UC1': 1,
            'rgb8': 3,
            'bgr8': 3,
            '8UC3': 3,
            'rgba8': 4,
            'bgra8': 4,
        }
        channels = channel_map.get(msg.encoding)
        if channels is None:
            raise ValueError(
                f'Unsupported image encoding for ArUco detection: '
                f'{msg.encoding}'
            )

        row_stride = int(msg.step)
        min_row_stride = int(msg.width) * channels
        if row_stride < min_row_stride:
            raise ValueError(
                f'Invalid image step {row_stride} for encoding '
                f'{msg.encoding} and width {msg.width}'
            )

        expected_bytes = int(msg.height) * row_stride
        frame_buffer = np.frombuffer(msg.data, dtype=np.uint8)
        if frame_buffer.size < expected_bytes:
            raise ValueError(
                f'Image buffer too small: got {frame_buffer.size} bytes, '
                f'expected at least {expected_bytes}'
            )

        row_view = frame_buffer[:expected_bytes].reshape(
            (msg.height, row_stride)
        )
        pixel_view = row_view[:, :min_row_stride]

        if channels == 1:
            return np.ascontiguousarray(
                pixel_view.reshape((msg.height, msg.width))
            )

        return np.ascontiguousarray(
            pixel_view.reshape((msg.height, msg.width, channels))
        )

    def _image_to_bgr(self, msg: Image) -> np.ndarray | None:
        """Convert a ROS Image to an OpenCV BGR image."""

        try:
            frame = self._ros_image_to_numpy(msg)
        except ValueError as error:
            self.get_logger().error(str(error))
            return None

        if msg.encoding in {'bgr8', '8UC3'}:
            return frame
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding in {'mono8', '8UC1'}:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if msg.encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if msg.encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        self.get_logger().error(
            f'Unsupported image encoding for ArUco detection: {msg.encoding}'
        )
        return None

    def _infer_camera_frame(self, topic_name: str) -> str:
        """Try to infer the camera frame name from the Gazebo topic path."""

        match = MODEL_TOPIC_PATTERN.match(topic_name)
        if match is not None:
            return match.group('link')
        return ''

    def _model_name_from_topic(self, topic_name: str) -> str:
        """Extract the Gazebo model name from a bridged camera topic."""

        match = MODEL_TOPIC_PATTERN.match(topic_name)
        if match is not None:
            return match.group('model')
        return ''

    def _world_name_from_topic(self, topic_name: str) -> str:
        """Extract the Gazebo world name from a bridged camera topic."""

        match = MODEL_TOPIC_PATTERN.match(topic_name)
        if match is not None:
            return match.group('world')
        return ''

    def _warn_throttled(self, message: str, warning_key: str):
        """Throttle repetitive warnings to keep logs readable."""

        now_ns = self.get_clock().now().nanoseconds
        warning_interval_ns = int(2.0e9)
        attribute_name = (
            '_warning_' + re.sub(r'[^a-zA-Z0-9_]', '_', warning_key) + '_ns'
        )
        last_warning_ns = getattr(self, attribute_name, 0)
        if now_ns - last_warning_ns < warning_interval_ns:
            return

        setattr(self, attribute_name, now_ns)
        self.get_logger().warn(message)


def main(args=None):
    """Run the ArUco detector node."""

    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
