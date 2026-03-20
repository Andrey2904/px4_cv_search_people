"""ROS2 node for the PX4 offboard waypoint mission."""

from enum import Enum
import math

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

from offboard_takeoff.mission import MissionPlan
from offboard_takeoff.mission import Waypoint
from offboard_takeoff.mission import default_mission_plan
from offboard_takeoff.navigation import TargetState
from offboard_takeoff.navigation import is_position_reached
from offboard_takeoff.navigation import is_yaw_reached
from offboard_takeoff.navigation import smooth_target_towards_waypoint


class MissionState(Enum):
    """Mission states for the PX4 offboard controller."""

    INIT = 'INIT'
    WAIT_FOR_PX4 = 'WAIT_FOR_PX4'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    HOLD = 'HOLD'
    RETURN_HOME = 'RETURN_HOME'
    LAND = 'LAND'
    FAILSAFE = 'FAILSAFE'
    FINISHED = 'FINISHED'


class OffboardTakeoff(Node):
    """Run a PX4 offboard mission through explicit mission states."""

    def __init__(self):
        super().__init__('offboard_takeoff')

        self._declare_parameters()
        self._load_parameters()

        self.mission_plan: MissionPlan = default_mission_plan(
            self.takeoff_height
        )
        self.required_initial_setpoints = 30

        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_status: VehicleStatus | None = None
        self.home_position: Waypoint | None = None
        self.return_home_waypoint: Waypoint | None = None

        self.node_started_at = self.get_clock().now()
        self.state_entered_at = self.node_started_at
        self.last_local_position_received_at = None
        self.last_vehicle_status_received_at = None
        self.last_vision_received_at = None
        self.active_waypoint_started_at = None
        self.last_offboard_request_at = None

        self.state = MissionState.INIT
        self.failure_reason = ''
        self.initial_setpoint_counter = 0
        self.current_search_waypoint_index = None
        self.landing_command_sent = False

        self.target = self._target_from_waypoint(
            self.mission_plan.takeoff_waypoint
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

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.timer = self.create_timer(self.control_period, self.control_loop)
        self.get_logger().info('OffboardTakeoff node started')
        self.get_logger().info('Waiting for initial PX4 data...')

    def _declare_parameters(self):
        """Declare ROS2 parameters for mission tuning and safety."""

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('takeoff_height', 3.0)
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('waypoint_tolerance', 0.4)
        self.declare_parameter('yaw_tolerance', 0.3)
        self.declare_parameter('px4_data_timeout_sec', 1.0)
        self.declare_parameter('waypoint_timeout_sec', 45.0)
        self.declare_parameter('arming_timeout_sec', 5.0)
        self.declare_parameter('hold_timeout_margin_sec', 1.0)
        self.declare_parameter('return_home_on_failure', True)
        self.declare_parameter('auto_land_on_finish', True)
        self.declare_parameter('enable_vision', False)
        self.declare_parameter('vision_timeout_sec', 1.0)

    def _load_parameters(self):
        """Load ROS2 parameters into node fields."""

        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.control_period = 1.0 / max(self.control_rate_hz, 1e-3)
        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.px4_data_timeout_sec = float(
            self.get_parameter('px4_data_timeout_sec').value
        )
        self.waypoint_timeout_sec = float(
            self.get_parameter('waypoint_timeout_sec').value
        )
        self.arming_timeout_sec = float(
            self.get_parameter('arming_timeout_sec').value
        )
        self.hold_timeout_margin_sec = float(
            self.get_parameter('hold_timeout_margin_sec').value
        )
        self.return_home_on_failure = bool(
            self.get_parameter('return_home_on_failure').value
        )
        self.auto_land_on_finish = bool(
            self.get_parameter('auto_land_on_finish').value
        )
        self.enable_vision = bool(self.get_parameter('enable_vision').value)
        self.vision_timeout_sec = float(
            self.get_parameter('vision_timeout_sec').value
        )

    @property
    def current_search_waypoint(self) -> Waypoint | None:
        """Return the currently active search waypoint."""

        if self.current_search_waypoint_index is None:
            return None

        if not 0 <= self.current_search_waypoint_index < len(
            self.mission_plan.search_waypoints
        ):
            return None

        return self.mission_plan.search_waypoints[
            self.current_search_waypoint_index
        ]

    @property
    def active_waypoint(self) -> Waypoint | None:
        """Return the active waypoint for the current state."""

        if self.state == MissionState.TAKEOFF:
            return self.mission_plan.takeoff_waypoint
        if self.state in {MissionState.SEARCH, MissionState.HOLD}:
            return self.current_search_waypoint
        if self.state == MissionState.RETURN_HOME:
            return self.return_home_waypoint
        return None

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Store the latest local position estimate."""

        self.vehicle_local_position = msg
        self.last_local_position_received_at = self.get_clock().now()

    def vehicle_status_callback(self, msg: VehicleStatus):
        """Store the latest PX4 vehicle status."""

        self.vehicle_status = msg
        self.last_vehicle_status_received_at = self.get_clock().now()

    def mark_vision_update(self):
        """Update the optional vision timestamp for future integrations."""

        self.last_vision_received_at = self.get_clock().now()

    def timestamp_us(self) -> int:
        """Return current ROS time in microseconds."""

        return self.get_clock().now().nanoseconds // 1000

    def age_sec(self, stamp) -> float:
        """Return age of a stored ROS timestamp in seconds."""

        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def time_in_state(self) -> float:
        """Return how long the node has been in the current state."""

        return self.age_sec(self.state_entered_at)

    def set_state(self, new_state: MissionState, reason: str = ''):
        """Transition to a new mission state with optional logging."""

        if self.state == new_state:
            return

        old_state = self.state
        self.state = new_state
        self.state_entered_at = self.get_clock().now()

        if new_state not in {
            MissionState.TAKEOFF,
            MissionState.SEARCH,
            MissionState.HOLD,
            MissionState.RETURN_HOME,
        }:
            self.active_waypoint_started_at = None

        if new_state == MissionState.LAND:
            self.landing_command_sent = False

        log_message = f'State {old_state.value} -> {new_state.value}'
        if reason:
            log_message += f' | {reason}'
        self.get_logger().info(log_message)

    def check_safety(self) -> bool:
        """Run centralized mission safety checks."""

        if self.state in {
            MissionState.INIT,
            MissionState.LAND,
            MissionState.FAILSAFE,
            MissionState.FINISHED,
        }:
            return True

        node_age_sec = self.age_sec(self.node_started_at)

        if self.last_local_position_received_at is None:
            if node_age_sec > self.px4_data_timeout_sec:
                return self.handle_failure(
                    'Local position data was not received'
                )
        elif (
            self.age_sec(self.last_local_position_received_at)
            > self.px4_data_timeout_sec
        ):
            return self.handle_failure('Local position data timeout')

        if self.last_vehicle_status_received_at is not None and (
            self.age_sec(self.last_vehicle_status_received_at)
            > self.px4_data_timeout_sec
        ):
            return self.handle_failure('PX4 status data timeout')

        if self.vehicle_status is not None and self.vehicle_status.failsafe:
            return self.handle_failure('PX4 reported failsafe state')

        if (
            self.state in {
                MissionState.TAKEOFF,
                MissionState.SEARCH,
                MissionState.RETURN_HOME,
            }
            and self.active_waypoint_started_at is not None
            and self.age_sec(self.active_waypoint_started_at)
            > self.waypoint_timeout_sec
        ):
            return self.handle_failure(
                f'{self.state.value} waypoint timeout exceeded'
            )

        if (
            self.state == MissionState.HOLD
            and self.current_search_waypoint is not None
        ):
            hold_limit = (
                self.current_search_waypoint.hold_time
                + self.hold_timeout_margin_sec
            )
            if self.time_in_state() > hold_limit:
                return self.handle_failure('Waypoint hold timeout exceeded')

        if (
            self.enable_vision
            and self.last_vision_received_at is not None
            and self.state in {MissionState.SEARCH, MissionState.HOLD}
            and self.age_sec(self.last_vision_received_at)
            > self.vision_timeout_sec
        ):
            return self.handle_failure('Vision data timeout')

        return True

    def handle_failure(self, reason: str) -> bool:
        """Handle a mission failure via return-home or passive failsafe."""

        if self.state == MissionState.FINISHED:
            return False

        if (
            self.state == MissionState.FAILSAFE
            and self.failure_reason == reason
        ):
            return False

        self.failure_reason = reason
        self.get_logger().error(f'Safety failure: {reason}')

        if self._can_return_home_after_failure():
            self._prepare_return_home_waypoint()
            self.set_state(
                MissionState.RETURN_HOME,
                f'Failure recovery: {reason}',
            )
            return False

        self._hold_current_position_target()
        self.set_state(MissionState.FAILSAFE, reason)
        return False

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

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ):
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

    def control_loop(self):
        """Run the mission state machine."""

        if self.state == MissionState.FINISHED:
            return

        safety_ok = self.check_safety()
        if not safety_ok and self.state == MissionState.FINISHED:
            return

        handlers = {
            MissionState.INIT: self.handle_init,
            MissionState.WAIT_FOR_PX4: self.handle_wait_for_px4,
            MissionState.ARMING: self.handle_arming,
            MissionState.TAKEOFF: self.handle_takeoff,
            MissionState.SEARCH: self.handle_search,
            MissionState.HOLD: self.handle_hold,
            MissionState.RETURN_HOME: self.handle_return_home,
            MissionState.LAND: self.handle_land,
            MissionState.FAILSAFE: self.handle_failsafe,
        }
        handlers[self.state]()

        if self._should_publish_offboard_setpoints():
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

    def handle_init(self):
        """Wait for valid initial data and capture home position."""

        if not self._has_valid_local_position():
            return

        if self.home_position is None:
            self.home_position = self._make_home_position()
            self.get_logger().info(
                'Home position saved -> x=%.2f y=%.2f z=%.2f yaw=%.2f'
                % (
                    self.home_position.x,
                    self.home_position.y,
                    self.home_position.z,
                    self.home_position.yaw,
                )
            )

        self.target = self._target_from_waypoint(
            self.mission_plan.takeoff_waypoint
        )
        self.initial_setpoint_counter = 0
        self.set_state(MissionState.WAIT_FOR_PX4, 'Initial position is ready')

    def handle_wait_for_px4(self):
        """Publish initial setpoints before requesting offboard mode."""

        self.target = self._target_from_waypoint(
            self.mission_plan.takeoff_waypoint
        )

        if self.initial_setpoint_counter < self.required_initial_setpoints:
            self.initial_setpoint_counter += 1
            if self.initial_setpoint_counter in {1, 10, 20, 30}:
                self.get_logger().info(
                    'Sending initial setpoints: %d/%d'
                    % (
                        self.initial_setpoint_counter,
                        self.required_initial_setpoints,
                    )
                )
            return

        self._request_offboard_and_arm()
        self.set_state(MissionState.ARMING, 'Initial offboard setpoints sent')

    def handle_arming(self):
        """Wait until PX4 confirms armed and offboard state."""

        self.target = self._target_from_waypoint(
            self.mission_plan.takeoff_waypoint
        )

        if self._is_armed() and self._is_offboard_active():
            self.active_waypoint_started_at = self.get_clock().now()
            self.set_state(
                MissionState.TAKEOFF,
                'Vehicle armed and offboard active',
            )
            return

        if self.last_vehicle_status_received_at is None and (
            self.time_in_state() >= self.arming_timeout_sec
        ):
            self.get_logger().warning(
                'PX4 status not available, continuing with legacy arm fallback'
            )
            self.active_waypoint_started_at = self.get_clock().now()
            self.set_state(
                MissionState.TAKEOFF,
                'Arming fallback without PX4 status confirmation',
            )
            return

        if (
            self.last_offboard_request_at is None
            or self.age_sec(self.last_offboard_request_at) >= 1.0
        ):
            self._request_offboard_and_arm()

    def handle_takeoff(self):
        """Fly to the configured working altitude."""

        self._update_smoothed_target()

        if not self._has_reached_active_waypoint(check_yaw=True):
            return

        self.get_logger().info('Takeoff waypoint reached')

        if not self.mission_plan.search_waypoints:
            self._complete_mission()
            return

        self._select_search_waypoint(0)
        self.set_state(
            MissionState.SEARCH,
            'Takeoff complete, starting search',
        )

    def handle_search(self):
        """Track the current mission waypoint."""

        waypoint = self.current_search_waypoint
        if waypoint is None:
            self._complete_mission()
            return

        self._update_smoothed_target()

        if not self._has_reached_active_waypoint(check_yaw=True):
            return

        self.get_logger().info(
            'Search waypoint %d/%d reached'
            % (
                self.current_search_waypoint_index + 1,
                len(self.mission_plan.search_waypoints),
            )
        )

        if waypoint.hold_time > 0.0:
            self.set_state(
                MissionState.HOLD,
                f'Holding current waypoint for {waypoint.hold_time:.1f}s',
            )
            return

        self._advance_search_or_finish()

    def handle_hold(self):
        """Hold the current waypoint for the configured duration."""

        if self.current_search_waypoint is None:
            self._complete_mission()
            return

        self._update_smoothed_target()

        if self.time_in_state() >= self.current_search_waypoint.hold_time:
            self._advance_search_or_finish()

    def handle_return_home(self):
        """Fly back to the saved home position before landing."""

        if self.return_home_waypoint is None:
            if self.home_position is None:
                self.set_state(
                    MissionState.FAILSAFE,
                    'Return-home requested without home position',
                )
                return
            self._prepare_return_home_waypoint()

        self._update_smoothed_target()

        if self._has_reached_active_waypoint(check_yaw=False):
            self.set_state(MissionState.LAND, 'Home position reached')

    def handle_land(self):
        """Land via PX4 command and wait until the vehicle disarms."""

        if not self.landing_command_sent:
            self.land()
            self.landing_command_sent = True
            self.get_logger().info('Landing in progress...')

        if self.vehicle_status is not None and not self._is_armed():
            self.set_state(
                MissionState.FINISHED,
                'Vehicle disarmed after landing',
            )

    def handle_failsafe(self):
        """Hold the current target without aggressive recovery actions."""

        if self.vehicle_status is not None and not self._is_armed():
            self.set_state(
                MissionState.FINISHED,
                'Vehicle disarmed in failsafe',
            )

    def _should_publish_offboard_setpoints(self) -> bool:
        """Return whether the current state needs offboard setpoints."""

        return self.state in {
            MissionState.WAIT_FOR_PX4,
            MissionState.ARMING,
            MissionState.TAKEOFF,
            MissionState.SEARCH,
            MissionState.HOLD,
            MissionState.RETURN_HOME,
            MissionState.FAILSAFE,
        }

    def _target_from_waypoint(self, waypoint: Waypoint) -> TargetState:
        """Convert a waypoint into a trajectory target."""

        return TargetState(
            x=waypoint.x,
            y=waypoint.y,
            z=waypoint.z,
            yaw=waypoint.yaw,
        )

    def _has_valid_local_position(self) -> bool:
        """Check whether local position data is available and valid."""

        if self.vehicle_local_position is None:
            return False

        return bool(
            self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
        )

    def _make_home_position(self) -> Waypoint:
        """Build a home waypoint from the current local position."""

        heading = self.vehicle_local_position.heading
        if not math.isfinite(heading):
            heading = 0.0

        return Waypoint(
            x=self.vehicle_local_position.x,
            y=self.vehicle_local_position.y,
            z=self.vehicle_local_position.z,
            yaw=heading,
        )

    def _update_smoothed_target(self):
        """Move the commanded setpoint toward the active waypoint."""

        waypoint = self.active_waypoint
        if waypoint is None:
            return

        self.target = smooth_target_towards_waypoint(
            self.target,
            waypoint,
            control_period=self.control_period,
            max_speed=self.max_speed,
            max_yaw_rate=self.max_yaw_rate,
        )

    def _has_reached_active_waypoint(self, check_yaw: bool) -> bool:
        """Check whether the vehicle has reached the active waypoint."""

        waypoint = self.active_waypoint
        if waypoint is None or self.vehicle_local_position is None:
            return False

        position_reached = is_position_reached(
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z,
            waypoint,
            self.waypoint_tolerance,
        )
        if not position_reached:
            return False

        if not check_yaw:
            return True

        return is_yaw_reached(
            self.vehicle_local_position.heading,
            waypoint.yaw,
            self.yaw_tolerance,
        )

    def _request_offboard_and_arm(self):
        """Request PX4 offboard mode and arm the vehicle."""

        self.engage_offboard_mode()
        self.arm()
        self.last_offboard_request_at = self.get_clock().now()

    def _is_armed(self) -> bool:
        """Check whether PX4 reports the vehicle as armed."""

        return (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    def _is_offboard_active(self) -> bool:
        """Check whether PX4 reports offboard mode as active."""

        return (
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _log_search_waypoint(self):
        """Log the active search waypoint for visibility."""

        waypoint = self.current_search_waypoint
        if waypoint is None:
            return

        self.get_logger().info(
            'Search waypoint %d/%d -> x=%.1f y=%.1f z=%.1f yaw=%.2f '
            'hold=%.1fs'
            % (
                self.current_search_waypoint_index + 1,
                len(self.mission_plan.search_waypoints),
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.yaw,
                waypoint.hold_time,
            )
        )

    def _select_search_waypoint(self, index: int):
        """Select the active search waypoint and reset its timeout."""

        self.current_search_waypoint_index = index
        self.active_waypoint_started_at = self.get_clock().now()
        self._log_search_waypoint()

    def _advance_search_or_finish(self):
        """Advance to the next search waypoint or finish the mission."""

        if self.current_search_waypoint_index is None:
            self._complete_mission()
            return

        next_index = self.current_search_waypoint_index + 1
        if next_index < len(self.mission_plan.search_waypoints):
            self._select_search_waypoint(next_index)
            self.set_state(
                MissionState.SEARCH,
                'Advancing to next search waypoint',
            )
            return

        self._complete_mission()

    def _complete_mission(self):
        """Finish the search mission according to the configured policy."""

        if not self.auto_land_on_finish:
            self.set_state(
                MissionState.FINISHED,
                'Mission completed without auto-land',
            )
            return

        if self.home_position is not None:
            self._prepare_return_home_waypoint()
            self.set_state(
                MissionState.RETURN_HOME,
                'Mission completed, returning home',
            )
            return

        self.set_state(
                MissionState.LAND,
                'Mission completed, landing in place',
            )

    def _prepare_return_home_waypoint(self):
        """Create a return-home target at home XY and mission altitude."""

        if self.home_position is None:
            self.return_home_waypoint = None
            return

        if self.target is not None:
            return_yaw = self.target.yaw
        else:
            return_yaw = self.home_position.yaw

        self.return_home_waypoint = Waypoint(
            x=self.home_position.x,
            y=self.home_position.y,
            z=self.mission_plan.takeoff_waypoint.z,
            yaw=return_yaw,
        )
        self.active_waypoint_started_at = self.get_clock().now()
        self.get_logger().info(
            'Return-home target -> x=%.2f y=%.2f z=%.2f yaw=%.2f'
            % (
                self.return_home_waypoint.x,
                self.return_home_waypoint.y,
                self.return_home_waypoint.z,
                self.return_home_waypoint.yaw,
            )
        )

    def _hold_current_position_target(self):
        """Freeze the current target close to the latest known vehicle pose."""

        if self.vehicle_local_position is None:
            return

        heading = self.vehicle_local_position.heading
        if not math.isfinite(heading):
            heading = self.target.yaw

        self.target = TargetState(
            x=self.vehicle_local_position.x,
            y=self.vehicle_local_position.y,
            z=self.vehicle_local_position.z,
            yaw=heading,
        )

    def _can_return_home_after_failure(self) -> bool:
        """Check whether return-home recovery is currently safe to attempt."""

        return (
            self.return_home_on_failure
            and self.home_position is not None
            and self._has_valid_local_position()
            and self.age_sec(self.last_local_position_received_at)
            <= self.px4_data_timeout_sec
            and self.age_sec(self.last_vehicle_status_received_at)
            <= self.px4_data_timeout_sec
            and self._is_armed()
        )
