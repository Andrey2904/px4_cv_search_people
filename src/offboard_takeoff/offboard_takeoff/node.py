"""ROS2 node for the PX4 offboard waypoint mission."""

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from offboard_takeoff.mission import default_mission_config
from offboard_takeoff.navigation import has_reached_waypoint
from offboard_takeoff.navigation import smooth_target_towards_waypoint
from offboard_takeoff.navigation import TargetState


class OffboardTakeoff(Node):
    """Run a simple PX4 offboard mission through predefined waypoints."""

    def __init__(self):
        super().__init__('offboard_takeoff')

        self.mission_config = default_mission_config()
        self.vehicle_local_position = None
        self.vehicle_status = None
        self.state = 'WAIT_FOR_POSITION'
        self.offboard_setpoint_counter = 0
        self.current_waypoint_index = 0
        self.landing_command_sent = False
        self.waypoint_hold_until_us = None

        first_waypoint = self.current_waypoint
        self.target = TargetState(
            x=first_waypoint.x,
            y=first_waypoint.y,
            z=first_waypoint.z,
            yaw=first_waypoint.yaw,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            px4_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.log_current_waypoint()
        self.timer = self.create_timer(
            self.mission_config.control_period,
            self.timer_callback,
        )
        self.get_logger().info('OffboardTakeoff node started')

    @property
    def current_waypoint(self):
        """Return the currently active mission waypoint."""

        return self.mission_config.waypoints[self.current_waypoint_index]

    def vehicle_local_position_callback(self, msg):
        """Store the latest local position estimate."""

        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Store the latest PX4 vehicle status."""

        self.vehicle_status = msg

    def timestamp_us(self):
        """Return current ROS time in microseconds."""

        return self.get_clock().now().nanoseconds // 1000

    def log_current_waypoint(self):
        """Log the current target waypoint for visibility."""

        waypoint = self.current_waypoint
        self.get_logger().info(
            'Target waypoint %d/%d -> x=%.1f y=%.1f z=%.1f yaw=%.2f hold=%.1fs'
            % (
                self.current_waypoint_index + 1,
                len(self.mission_config.waypoints),
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.yaw,
                waypoint.hold_time,
            )
        )

    def start_waypoint_hold(self):
        """Start the optional pause after reaching a waypoint."""

        hold_time_us = int(self.current_waypoint.hold_time * 1_000_000)
        self.waypoint_hold_until_us = self.timestamp_us() + hold_time_us
        self.state = 'HOLD_WAYPOINT'
        self.get_logger().info(
            f'Holding waypoint for {self.current_waypoint.hold_time:.1f}s'
        )

    def advance_to_next_waypoint_or_land(self):
        """Advance mission to the next waypoint or land if mission is finished."""

        self.waypoint_hold_until_us = None

        if self.current_waypoint_index + 1 < len(self.mission_config.waypoints):
            self.current_waypoint_index += 1
            self.log_current_waypoint()
            self.state = 'FLY_WAYPOINTS'
            return

        self.state = 'LAND'

    def update_smoothed_target(self):
        """Move the commanded setpoint gradually toward the active waypoint."""

        self.target = smooth_target_towards_waypoint(
            self.target,
            self.current_waypoint,
            self.mission_config,
        )

    def has_reached_target(self):
        """Check whether the vehicle reached the active waypoint."""

        if self.vehicle_local_position is None:
            return False

        return has_reached_waypoint(
            current_x=self.vehicle_local_position.x,
            current_y=self.vehicle_local_position.y,
            current_z=self.vehicle_local_position.z,
            current_yaw=self.vehicle_local_position.heading,
            waypoint=self.current_waypoint,
            config=self.mission_config,
        )

    def publish_offboard_control_mode(self):
        """Publish PX4 offboard control mode."""

        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish the smoothed position setpoint."""

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [
            float(self.target.x),
            float(self.target.y),
            float(self.target.z),
        ]
        msg.yaw = float(self.target.yaw)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a PX4 vehicle command."""

        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """Send arm command."""

        self.get_logger().info('ARM command')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self):
        """Send disarm command."""

        self.get_logger().info('DISARM command')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    def engage_offboard_mode(self):
        """Switch PX4 to offboard mode."""

        self.get_logger().info('OFFBOARD mode command')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def land(self):
        """Send land command."""

        self.get_logger().info('LAND command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_callback(self):
        """Run the mission state machine."""

        self.update_smoothed_target()
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.state == 'WAIT_FOR_POSITION':
            if self.vehicle_local_position is None:
                self.get_logger().info('Waiting for vehicle_local_position...')
                return

            self.get_logger().info('Vehicle local position received')
            self.state = 'SEND_SETPOINTS'
            return

        if self.state == 'SEND_SETPOINTS':
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(
                    'Sending initial setpoints: '
                    f'{self.offboard_setpoint_counter}/30'
                )

            if self.offboard_setpoint_counter >= 30:
                self.state = 'ENABLE_OFFBOARD'
            return

        if self.state == 'ENABLE_OFFBOARD':
            self.engage_offboard_mode()
            self.state = 'ARM'
            return

        if self.state == 'ARM':
            self.arm()
            self.state = 'FLY_WAYPOINTS'
            return

        if self.state == 'FLY_WAYPOINTS':
            if not self.has_reached_target():
                return

            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached'
            )

            if self.current_waypoint.hold_time > 0.0:
                self.start_waypoint_hold()
                return

            self.advance_to_next_waypoint_or_land()
            return

        if self.state == 'HOLD_WAYPOINT':
            if self.waypoint_hold_until_us is None:
                self.advance_to_next_waypoint_or_land()
                return

            if self.timestamp_us() >= self.waypoint_hold_until_us:
                self.advance_to_next_waypoint_or_land()
            return

        if self.state == 'LAND':
            if not self.landing_command_sent:
                self.land()
                self.landing_command_sent = True
                self.get_logger().info('Mission complete, landing...')
