"""Console entry point for the PX4 offboard mission node."""

import rclpy

from offboard_takeoff.node import OffboardTakeoff


def main(args=None):
    """Start the ROS2 offboard mission node."""

    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
