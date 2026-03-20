"""Console entry point for the PX4 offboard mission node."""

import time

import rclpy

from offboard_takeoff.node import OffboardTakeoff


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt -> landing')
        node.timer.cancel()
        node.land()
        time.sleep(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
