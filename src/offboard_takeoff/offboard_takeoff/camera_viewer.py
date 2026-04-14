"""ROS 2 camera viewer and relay with YOLO target overlay."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

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
        self.declare_parameter('enable_target_overlay', True)
        self.declare_parameter('target_bbox_topic', 'yolo/target_bbox')

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
        self.enable_target_overlay = bool(
            self.get_parameter('enable_target_overlay').value
        )
        self.target_bbox_topic = str(
            self.get_parameter('target_bbox_topic').value
        )
        self.camera_info_logged = False
        self.relay_frame_count = 0
        self.latest_target_bbox: tuple[int, int, int, int, int, int, int] | None = None

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
        self.target_bbox_subscription = self.create_subscription(
            Int32MultiArray,
            self.target_bbox_topic,
            self.target_bbox_callback,
            10,
        )

        self.get_logger().info(
            'Camera viewer relay started: '
            f'in_image={self.image_topic}, '
            f'in_camera_info={self.camera_info_topic}, '
            f'out_image={self.relay_image_topic}, '
            f'out_camera_info={self.relay_camera_info_topic}, '
            f'show_window={self.show_window}, '
            f'enable_target_overlay={self.enable_target_overlay}'
        )

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
        if self.enable_target_overlay:
            display_frame = self.draw_target_overlay(display_frame)

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

    def target_bbox_callback(self, msg: Int32MultiArray):
        """Store the latest target bbox for display overlays."""

        data = list(msg.data)
        if len(data) < 7:
            self.latest_target_bbox = None
            return

        self.latest_target_bbox = (
            int(data[0]),
            int(data[1]),
            int(data[2]),
            int(data[3]),
            int(data[4]),
            max(int(data[5]), 1),
            max(int(data[6]), 1),
        )

    def draw_target_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Draw the latest detector target over the camera preview."""

        if self.latest_target_bbox is None:
            return frame

        confidence_milli, center_x, center_y, width, height, _, _ = (
            self.latest_target_bbox
        )
        if width <= 1 or height <= 1:
            return frame

        x1 = max(int(round(center_x - width / 2.0)), 0)
        y1 = max(int(round(center_y - height / 2.0)), 0)
        x2 = min(int(round(center_x + width / 2.0)), frame.shape[1] - 1)
        y2 = min(int(round(center_y + height / 2.0)), frame.shape[0] - 1)
        if x2 <= x1 or y2 <= y1:
            return frame

        color = (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.circle(frame, (center_x, center_y), 4, color, -1)

        label = f'PERSON {confidence_milli / 1000.0:.2f}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            label,
            font,
            font_scale,
            thickness,
        )
        badge_x1 = x1
        badge_y2 = max(y1 - 6, text_height + baseline + 8)
        badge_y1 = max(badge_y2 - text_height - baseline - 10, 0)
        badge_x2 = min(
            badge_x1 + text_width + 14,
            frame.shape[1] - 1,
        )

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (badge_x1, badge_y1),
            (badge_x2, badge_y2),
            color,
            -1,
        )
        cv2.addWeighted(overlay, 0.85, frame, 0.15, 0.0, frame)
        cv2.putText(
            frame,
            label,
            (badge_x1 + 7, badge_y2 - baseline - 4),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )
        return frame

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
