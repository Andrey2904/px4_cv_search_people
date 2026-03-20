"""ROS2 node that displays the drone camera stream with OpenCV."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


CAMERA_TOPIC = (
    '/world/my_world_2/model/x500_mono_cam_0/'
    'link/camera_link/sensor/camera/image'
)


class CameraViewer(Node):
    """Subscribe to the drone camera and render frames with OpenCV."""

    def __init__(self):
        super().__init__('camera_viewer')

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10,
        )

        self.get_logger().info(f'Listening camera topic: {CAMERA_TOPIC}')

    def image_callback(self, msg):
        """Convert ROS image to OpenCV frame and display it."""

        frame = self.image_to_bgr(msg)
        gray, blurred, edges = self.process_frame(frame)

        cv2.imshow('Drone Camera', frame)
        cv2.imshow('Drone Camera Gray', gray)
        cv2.imshow('Drone Camera Edges', edges)
        cv2.waitKey(1)

    def image_to_bgr(self, msg):
        """Convert sensor_msgs/Image to an OpenCV BGR frame."""

        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            self.get_logger().error(f'Unsupported image encoding: {msg.encoding}')
            return np.zeros((1, 1, 3), dtype=np.uint8)

        channels = 1 if msg.encoding == 'mono8' else 3
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, channels))

        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if msg.encoding == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    def process_frame(self, frame):
        """Run a simple OpenCV pipeline for further experimentation."""

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 150)
        return gray, blurred, edges


def main(args=None):
    """Run the camera viewer node."""

    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping camera viewer')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
