"""ROS 2 camera viewer, relay, and simple ArUco overlay detector."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


ARUCO_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith('DICT_')
}


class CameraViewer(Node):
    """Subscribe to a camera stream, optionally display it, and relay it."""

    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('relay_image_topic', '/camera/image_raw')
        self.declare_parameter('relay_camera_info_topic', '/camera/camera_info')
        self.declare_parameter('show_window', True)
        self.declare_parameter('log_camera_info', True)
        self.declare_parameter('enable_aruco_overlay', True)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('draw_rejected_candidates', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.relay_image_topic = str(
            self.get_parameter('relay_image_topic').value
        )
        self.relay_camera_info_topic = str(
            self.get_parameter('relay_camera_info_topic').value
        )
        self.show_window = bool(self.get_parameter('show_window').value)
        self.log_camera_info = bool(
            self.get_parameter('log_camera_info').value
        )
        self.enable_aruco_overlay = bool(
            self.get_parameter('enable_aruco_overlay').value
        )
        self.aruco_dictionary_name = str(
            self.get_parameter('aruco_dictionary').value
        )
        self.draw_rejected_candidates = bool(
            self.get_parameter('draw_rejected_candidates').value
        )
        self.camera_info_logged = False
        self.relay_frame_count = 0
        self.last_detected_ids: tuple[int, ...] = ()

        self.image_publisher = self.create_publisher(
            Image,
            self.relay_image_topic,
            qos_profile_sensor_data,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.relay_camera_info_topic,
            qos_profile_sensor_data,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.aruco_dictionary = self._create_aruco_dictionary(
            self.aruco_dictionary_name
        )
        self.aruco_parameters = self._create_aruco_parameters()
        self.aruco_detector = self._create_aruco_detector()

        self.get_logger().info(
            'Camera viewer relay started: '
            f'in_image={self.image_topic}, '
            f'in_camera_info={self.camera_info_topic}, '
            f'out_image={self.relay_image_topic}, '
            f'out_camera_info={self.relay_camera_info_topic}, '
            f'show_window={self.show_window}, '
            f'enable_aruco_overlay={self.enable_aruco_overlay}, '
            f'aruco_dictionary={self.aruco_dictionary_name}'
        )

    def _create_aruco_dictionary(self, dictionary_name: str):
        """Create the configured OpenCV ArUco dictionary."""

        if dictionary_name not in ARUCO_DICTIONARIES:
            supported = ', '.join(sorted(ARUCO_DICTIONARIES))
            raise ValueError(
                f'Unsupported ArUco dictionary "{dictionary_name}". '
                f'Available dictionaries: {supported}'
            )

        return cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTIONARIES[dictionary_name]
        )

    def _create_aruco_parameters(self):
        """Build OpenCV ArUco detector parameters."""

        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _create_aruco_detector(self):
        """Instantiate the modern ArUco detector API when available."""

        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(
                self.aruco_dictionary,
                self.aruco_parameters,
            )
        return None

    def camera_info_callback(self, msg: CameraInfo):
        """Relay camera calibration and log it once for debugging."""

        self.camera_info_publisher.publish(msg)

        if self.log_camera_info and not self.camera_info_logged:
            self.camera_info_logged = True
            self.get_logger().info(
                'Received CameraInfo: '
                f'frame_id={msg.header.frame_id or "<empty>"}, '
                f'size={msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.3f}, fy={msg.k[4]:.3f}, '
                f'cx={msg.k[2]:.3f}, cy={msg.k[5]:.3f}'
            )

    def image_callback(self, msg: Image):
        """Relay the image and optionally show it with OpenCV."""

        self.image_publisher.publish(msg)
        self.relay_frame_count += 1

        frame = self.image_to_bgr(msg)
        if frame is None:
            return

        display_frame = frame.copy()
        if self.enable_aruco_overlay:
            display_frame = self.draw_aruco_overlay(display_frame)

        if self.show_window:
            cv2.imshow('Gazebo Camera Relay', display_frame)
            cv2.waitKey(1)

        if self.relay_frame_count == 1:
            self.get_logger().info(
                'Received first image frame: '
                f'encoding={msg.encoding}, '
                f'size={msg.width}x{msg.height}, '
                f'frame_id={msg.header.frame_id or "<empty>"}'
            )

    def draw_aruco_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Detect ArUco markers and draw their outlines and IDs."""

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detect_markers(gray)

        detected_ids: tuple[int, ...] = ()
        if ids is not None and len(ids) > 0:
            detected_ids = tuple(sorted(int(marker_id[0]) for marker_id in ids))
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_corners, marker_id_array in zip(corners, ids):
                marker_id = int(marker_id_array[0])
                center = np.mean(
                    marker_corners.reshape(4, 2),
                    axis=0,
                ).astype(int)
                cv2.putText(
                    frame,
                    f'id={marker_id}',
                    (int(center[0] + 10), int(center[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        if self.draw_rejected_candidates and rejected:
            cv2.aruco.drawDetectedMarkers(
                frame,
                rejected,
                borderColor=(0, 0, 255),
            )

        if detected_ids != self.last_detected_ids:
            if detected_ids:
                self.get_logger().info(
                    f'Detected ArUco markers in viewer: {list(detected_ids)}'
                )
            elif self.last_detected_ids:
                self.get_logger().info(
                    'Viewer lost sight of all ArUco markers'
                )
            self.last_detected_ids = detected_ids

        return frame

    def detect_markers(self, gray_frame: np.ndarray):
        """Run OpenCV ArUco detection using the available API."""

        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray_frame)

        return cv2.aruco.detectMarkers(
            gray_frame,
            self.aruco_dictionary,
            parameters=self.aruco_parameters,
        )

    def image_to_bgr(self, msg: Image) -> np.ndarray | None:
        """Convert sensor_msgs/Image to an OpenCV BGR frame."""

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
            self.get_logger().error(
                f'Unsupported image encoding: {msg.encoding}'
            )
            return None

        row_stride = int(msg.step)
        min_row_stride = int(msg.width) * channels
        if row_stride < min_row_stride:
            self.get_logger().error(
                f'Invalid image step {row_stride} for encoding '
                f'{msg.encoding} and width {msg.width}'
            )
            return None

        expected_bytes = int(msg.height) * row_stride
        frame_buffer = np.frombuffer(msg.data, dtype=np.uint8)
        if frame_buffer.size < expected_bytes:
            self.get_logger().error(
                f'Image buffer too small: got {frame_buffer.size} bytes, '
                f'expected at least {expected_bytes}'
            )
            return None

        row_view = frame_buffer[:expected_bytes].reshape(
            (msg.height, row_stride)
        )
        pixel_view = row_view[:, :min_row_stride]

        if channels == 1:
            frame = np.ascontiguousarray(
                pixel_view.reshape((msg.height, msg.width))
            )
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        frame = np.ascontiguousarray(
            pixel_view.reshape((msg.height, msg.width, channels))
        )
        if msg.encoding in {'bgr8', '8UC3'}:
            return frame
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if msg.encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame


def main(args=None):
    """Run the camera viewer node."""

    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping camera viewer relay')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
