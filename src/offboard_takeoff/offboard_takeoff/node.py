"""ROS2 node for the PX4 offboard waypoint mission."""

from dataclasses import dataclass
from enum import Enum
import math
import subprocess

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
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

from offboard_takeoff.mission import MissionPlan
from offboard_takeoff.mission import Waypoint
from offboard_takeoff.mission import default_mission_plan
from offboard_takeoff.navigation import TargetState
from offboard_takeoff.navigation import is_position_reached
from offboard_takeoff.navigation import is_yaw_reached
from offboard_takeoff.navigation import smooth_target_towards_waypoint
from offboard_takeoff.path_planner import EvacuationRoutePlanner
from offboard_takeoff.path_planner import PlannerConfig


class MissionState(Enum):
    """Mission states for the PX4 offboard controller."""

    INIT = 'INIT'
    WAIT_FOR_PX4 = 'WAIT_FOR_PX4'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    HOLD = 'HOLD'
    FOLLOW_PERSON = 'FOLLOW_PERSON'
    RETURN_HOME = 'RETURN_HOME'
    LAND = 'LAND'
    FAILSAFE = 'FAILSAFE'
    FINISHED = 'FINISHED'


@dataclass(frozen=True)
class PersonTarget:
    """Best detected person bounding box and image metadata."""

    confidence: float
    center_x: int
    center_y: int
    width: int
    height: int
    image_width: int
    image_height: int

    @property
    def center_x_error(self) -> float:
        """Normalized horizontal error relative to the image center."""

        if self.image_width <= 0:
            return 0.0
        return (
            float(self.center_x) - float(self.image_width) / 2.0
        ) / max(float(self.image_width) / 2.0, 1.0)

    @property
    def height_ratio(self) -> float:
        """Bounding box height normalized by image height."""

        if self.image_height <= 0:
            return 0.0
        return float(self.height) / float(self.image_height)

    @property
    def width_ratio(self) -> float:
        """Bounding box width normalized by image width."""

        if self.image_width <= 0:
            return 0.0
        return float(self.width) / float(self.image_width)

    @property
    def left_ratio(self) -> float:
        """Left bbox edge normalized by image width."""

        if self.image_width <= 0:
            return 0.0
        return (
            float(self.center_x) - float(self.width) / 2.0
        ) / float(self.image_width)

    @property
    def right_ratio(self) -> float:
        """Right bbox edge normalized by image width."""

        if self.image_width <= 0:
            return 1.0
        return (
            float(self.center_x) + float(self.width) / 2.0
        ) / float(self.image_width)

    @property
    def bottom_ratio(self) -> float:
        """Bottom bbox edge normalized by image height."""

        if self.image_height <= 0:
            return 1.0
        return (
            float(self.center_y) + float(self.height) / 2.0
        ) / float(self.image_height)


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
        self.evacuation_route_waypoints: list[Waypoint] = []
        self.evacuation_route_index = 0
        self.person_target: PersonTarget | None = None
        self.last_stable_person_target: PersonTarget | None = None

        self.node_started_at = self.get_clock().now()
        self.state_entered_at = self.node_started_at
        self.last_local_position_received_at = None
        self.last_vehicle_status_received_at = None
        self.last_vision_received_at = None
        self.active_waypoint_started_at = None
        self.last_offboard_request_at = None
        self.last_person_follow_log_at = self.node_started_at
        self.last_person_detection_log_at = None
        self.person_follow_started_at = None
        self.person_detection_ignore_until = None
        self.pending_search_resume_after_home = False
        self.pending_teleport_after_home = False
        self.rescue_model_teleported = False
        self.resume_search_waypoint_index = None
        self.person_action_phase = ''
        self.person_action_phase_started_at = None
        self.person_reference_bbox_height = 0.0
        self.person_detection_started_at = None
        self.person_detection_lost_at = None
        self.person_align_loss_started_at = None
        self.person_approach_loss_started_at = None
        self.last_approach_person_target: PersonTarget | None = None
        self.last_approach_person_target_at = None
        self.person_follow_confirmation_logged = False
        self.person_visible_since = None
        self.last_stable_person_target_at = None
        self.person_centered_since = None
        self.person_yaw_locked = False
        self.person_hold_yaw = None
        self.last_significant_person_center_x_error = 0.0
        self.filtered_person_center_x_error = 0.0
        self.filtered_follow_yaw_rate = 0.0
        self.previous_person_center_x_error = 0.0
        self.previous_yaw_error_sample_at = None

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
        self.evacuation_route_pub = self.create_publisher(
            Float32MultiArray,
            'mission/evacuation_route',
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
        self.create_subscription(
            Int32MultiArray,
            'yolo/target_bbox',
            self.person_target_callback,
            10,
        )

        self.timer = self.create_timer(self.control_period, self.control_loop)
        self.get_logger().info('OffboardTakeoff node started')
        self.get_logger().info('Waiting for initial PX4 data...')

    def _declare_parameters(self):
        """Declare ROS2 parameters for mission tuning and safety."""

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('takeoff_height', 7.0)
        self.declare_parameter('max_speed', 1.5)
        # Previous tuning: max_yaw_rate=0.4, follow_yaw_kp=0.9,
        # follow_align_dropout_continue_sec=1.0,
        # follow_target_memory_timeout_sec=1.0.
        self.declare_parameter('max_yaw_rate', 0.5)
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
        self.declare_parameter('enable_person_follow', True)
        self.declare_parameter('follow_min_confidence', 0.50)
        self.declare_parameter('follow_detection_confirm_sec', 1.0)
        self.declare_parameter('follow_detection_gap_tolerance_sec', 0.5)
        self.declare_parameter('post_rescue_detection_cooldown_sec', 5.0)
        self.declare_parameter('rescue_model_name', 'man_3')
        self.declare_parameter('rescue_model_teleport_x', 36.57)
        self.declare_parameter('rescue_model_teleport_y', 28.2)
        self.declare_parameter('rescue_model_teleport_z', -10.0)
        self.declare_parameter('rescue_service_timeout_ms', 2000)
        self.declare_parameter('teleport_world_name', 'forest')
        self.declare_parameter('follow_person_action_timeout_sec', 90.0)
        self.declare_parameter('follow_stop_before_approach_sec', 1.0)
        self.declare_parameter('follow_bbox_growth_target', 4.0)
        self.declare_parameter('follow_hover_after_approach_sec', 5.0)
        self.declare_parameter('follow_person_timeout_sec', 1.0)
        self.declare_parameter('follow_align_dropout_continue_sec', 1.5)
        self.declare_parameter('follow_target_memory_activation_sec', 1.0)
        self.declare_parameter('follow_target_memory_timeout_sec', 1.5)
        self.declare_parameter('follow_forward_speed', 0.35)
        self.declare_parameter('follow_slow_forward_speed', 0.15)
        self.declare_parameter('follow_stop_bbox_height_ratio', 0.33)
        self.declare_parameter('follow_slow_bbox_height_ratio', 0.22)
        self.declare_parameter('follow_frame_edge_margin_ratio', 0.08)
        self.declare_parameter('follow_bottom_edge_margin_ratio', 0.08)
        self.declare_parameter('follow_approach_dropout_stop_sec', 0.5)
        self.declare_parameter('follow_approach_max_sec', 12.0)
        self.declare_parameter('follow_yaw_kp', 0.8)
        self.declare_parameter('follow_yaw_kd', 0.0)
        self.declare_parameter('follow_yaw_deadband', 0.05)  # Previous: 0.05.
        self.declare_parameter('follow_yaw_unlock_deadband', 0.12)
        self.declare_parameter('follow_yaw_rate_filter_alpha', 0.35)
        self.declare_parameter('follow_lateral_gain', 0.35)
        self.declare_parameter('follow_lateral_deadband', 0.20)
        self.declare_parameter('follow_log_period_sec', 1.0)
        self.declare_parameter('follow_target_lead_limit', 1.5)
        self.declare_parameter('follow_alignment_hold_sec', 1.0)
        self.declare_parameter('follow_centered_hold_sec', 0.35)
        self.declare_parameter('follow_error_filter_alpha', 0.25)
        self.declare_parameter('enable_evacuation_astar', True)
        self.declare_parameter(
            'evacuation_world_sdf_path',
            '/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf',
        )
        self.declare_parameter('evacuation_grid_resolution', 1.0)
        self.declare_parameter('evacuation_obstacle_margin', 3.0)
        self.declare_parameter('evacuation_map_padding', 8.0)
        self.declare_parameter('evacuation_waypoint_spacing', 3.0)
        self.declare_parameter('evacuation_mission_rotation_deg', 90.0)
        self.declare_parameter('evacuation_mission_invert_y', True)
        self.declare_parameter('evacuation_max_iterations', 20000)

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
        self.enable_person_follow = bool(
            self.get_parameter('enable_person_follow').value
        )
        self.follow_min_confidence = float(
            self.get_parameter('follow_min_confidence').value
        )
        self.follow_detection_confirm_sec = float(
            self.get_parameter('follow_detection_confirm_sec').value
        )
        self.follow_detection_gap_tolerance_sec = float(
            self.get_parameter('follow_detection_gap_tolerance_sec').value
        )
        self.post_rescue_detection_cooldown_sec = float(
            self.get_parameter('post_rescue_detection_cooldown_sec').value
        )
        self.rescue_model_name = str(
            self.get_parameter('rescue_model_name').value
        ).strip()
        self.rescue_model_teleport_x = float(
            self.get_parameter('rescue_model_teleport_x').value
        )
        self.rescue_model_teleport_y = float(
            self.get_parameter('rescue_model_teleport_y').value
        )
        self.rescue_model_teleport_z = float(
            self.get_parameter('rescue_model_teleport_z').value
        )
        self.rescue_service_timeout_ms = max(
            int(self.get_parameter('rescue_service_timeout_ms').value),
            100,
        )
        self.teleport_world_name = str(
            self.get_parameter('teleport_world_name').value
        ).strip()
        self.follow_person_action_timeout_sec = float(
            self.get_parameter('follow_person_action_timeout_sec').value
        )
        self.follow_stop_before_approach_sec = float(
            self.get_parameter('follow_stop_before_approach_sec').value
        )
        self.follow_bbox_growth_target = float(
            self.get_parameter('follow_bbox_growth_target').value
        )
        self.follow_hover_after_approach_sec = float(
            self.get_parameter('follow_hover_after_approach_sec').value
        )
        self.follow_person_timeout_sec = float(
            self.get_parameter('follow_person_timeout_sec').value
        )
        self.follow_align_dropout_continue_sec = float(
            self.get_parameter('follow_align_dropout_continue_sec').value
        )
        self.follow_target_memory_activation_sec = float(
            self.get_parameter('follow_target_memory_activation_sec').value
        )
        self.follow_target_memory_timeout_sec = float(
            self.get_parameter('follow_target_memory_timeout_sec').value
        )
        self.follow_forward_speed = float(
            self.get_parameter('follow_forward_speed').value
        )
        self.follow_slow_forward_speed = float(
            self.get_parameter('follow_slow_forward_speed').value
        )
        self.follow_stop_bbox_height_ratio = float(
            self.get_parameter('follow_stop_bbox_height_ratio').value
        )
        self.follow_slow_bbox_height_ratio = float(
            self.get_parameter('follow_slow_bbox_height_ratio').value
        )
        self.follow_frame_edge_margin_ratio = float(
            self.get_parameter('follow_frame_edge_margin_ratio').value
        )
        self.follow_bottom_edge_margin_ratio = float(
            self.get_parameter('follow_bottom_edge_margin_ratio').value
        )
        self.follow_approach_dropout_stop_sec = float(
            self.get_parameter('follow_approach_dropout_stop_sec').value
        )
        self.follow_approach_max_sec = float(
            self.get_parameter('follow_approach_max_sec').value
        )
        self.follow_yaw_kp = float(
            self.get_parameter('follow_yaw_kp').value
        )
        self.follow_yaw_kd = float(
            self.get_parameter('follow_yaw_kd').value
        )
        self.follow_yaw_deadband = float(
            self.get_parameter('follow_yaw_deadband').value
        )
        self.follow_yaw_unlock_deadband = float(
            self.get_parameter('follow_yaw_unlock_deadband').value
        )
        self.follow_yaw_rate_filter_alpha = float(
            self.get_parameter('follow_yaw_rate_filter_alpha').value
        )
        self.follow_lateral_gain = float(
            self.get_parameter('follow_lateral_gain').value
        )
        self.follow_lateral_deadband = float(
            self.get_parameter('follow_lateral_deadband').value
        )
        self.follow_log_period_sec = float(
            self.get_parameter('follow_log_period_sec').value
        )
        self.follow_target_lead_limit = float(
            self.get_parameter('follow_target_lead_limit').value
        )
        self.follow_alignment_hold_sec = float(
            self.get_parameter('follow_alignment_hold_sec').value
        )
        self.follow_centered_hold_sec = float(
            self.get_parameter('follow_centered_hold_sec').value
        )
        self.follow_error_filter_alpha = float(
            self.get_parameter('follow_error_filter_alpha').value
        )
        self.enable_evacuation_astar = bool(
            self.get_parameter('enable_evacuation_astar').value
        )
        self.evacuation_world_sdf_path = str(
            self.get_parameter('evacuation_world_sdf_path').value
        ).strip()
        self.evacuation_grid_resolution = float(
            self.get_parameter('evacuation_grid_resolution').value
        )
        self.evacuation_obstacle_margin = float(
            self.get_parameter('evacuation_obstacle_margin').value
        )
        self.evacuation_map_padding = float(
            self.get_parameter('evacuation_map_padding').value
        )
        self.evacuation_waypoint_spacing = float(
            self.get_parameter('evacuation_waypoint_spacing').value
        )
        self.evacuation_mission_rotation_deg = float(
            self.get_parameter('evacuation_mission_rotation_deg').value
        )
        self.evacuation_mission_invert_y = bool(
            self.get_parameter('evacuation_mission_invert_y').value
        )
        self.evacuation_max_iterations = int(
            self.get_parameter('evacuation_max_iterations').value
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

    def person_target_callback(self, msg: Int32MultiArray):
        """Store the latest best-person bounding box from YOLO."""

        data = list(msg.data)
        if len(data) < 7:
            self.person_target = None
            self.person_visible_since = None
            self.last_person_detection_log_at = None
            return

        now = self.get_clock().now()
        if self.person_visible_since is None:
            self.person_visible_since = now

        self.person_target = PersonTarget(
            confidence=float(data[0]) / 1000.0,
            center_x=int(data[1]),
            center_y=int(data[2]),
            width=max(int(data[3]), 0),
            height=max(int(data[4]), 0),
            image_width=max(int(data[5]), 1),
            image_height=max(int(data[6]), 1),
        )
        alpha = min(max(self.follow_error_filter_alpha, 0.0), 1.0)
        raw_error = self.person_target.center_x_error
        if abs(raw_error) > self.follow_yaw_deadband:
            self.last_significant_person_center_x_error = raw_error
        self.filtered_person_center_x_error = (
            alpha * raw_error
            + (1.0 - alpha) * self.filtered_person_center_x_error
        )
        if (
            self.age_sec(self.person_visible_since)
            >= self.follow_target_memory_activation_sec
        ):
            self.last_stable_person_target = self.person_target
            self.last_stable_person_target_at = now
        if self.last_person_detection_log_at is None:
            self.get_logger().info(
                'Person bbox received: '
                f'confidence={self.person_target.confidence:.2f}, '
                'center=('
                f'{self.person_target.center_x}, '
                f'{self.person_target.center_y}), '
                f'size={self.person_target.width}x'
                f'{self.person_target.height}, '
                f'image={self.person_target.image_width}x'
                f'{self.person_target.image_height}, '
                f'x_error={raw_error:.3f}'
            )
            self.last_person_detection_log_at = now
        self.mark_vision_update()

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
            MissionState.FOLLOW_PERSON,
            MissionState.RETURN_HOME,
        }:
            self.active_waypoint_started_at = None
            self.person_follow_started_at = None
            self.person_action_phase = ''
            self.person_action_phase_started_at = None
            self.person_reference_bbox_height = 0.0
            self.person_detection_started_at = None
            self.person_detection_lost_at = None
            self.person_align_loss_started_at = None
            self.person_approach_loss_started_at = None
            self.last_approach_person_target = None
            self.last_approach_person_target_at = None
            self.person_follow_confirmation_logged = False
            self.person_visible_since = None
            self.person_centered_since = None
            self.person_yaw_locked = False
            self.person_hold_yaw = None
            self.last_significant_person_center_x_error = 0.0
            self.filtered_follow_yaw_rate = 0.0
            self.previous_yaw_error_sample_at = None

        if new_state == MissionState.LAND:
            self.landing_command_sent = False

        if new_state == MissionState.FOLLOW_PERSON:
            self.person_follow_started_at = self.get_clock().now()
            self.person_action_phase = 'STOP_BEFORE_APPROACH'
            self.person_action_phase_started_at = self.get_clock().now()
            self.person_reference_bbox_height = 0.0
            self.person_align_loss_started_at = None
            self.person_approach_loss_started_at = None
            self.last_approach_person_target = None
            self.last_approach_person_target_at = None
            self.person_follow_confirmation_logged = False
            self.person_centered_since = None
            self.person_yaw_locked = False
            self.person_hold_yaw = None
            self.filtered_follow_yaw_rate = 0.0
            self.previous_person_center_x_error = (
                self.filtered_person_center_x_error
            )
            self.previous_yaw_error_sample_at = self.get_clock().now()

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
            self.state == MissionState.FOLLOW_PERSON
            and self.person_follow_started_at is not None
            and self.age_sec(self.person_follow_started_at)
            > self.follow_person_action_timeout_sec
        ):
            return self.handle_failure(
                'FOLLOW_PERSON action timeout exceeded'
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
            MissionState.FOLLOW_PERSON: self.handle_follow_person,
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
            self._maybe_start_person_follow()
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

        if self._maybe_start_person_follow():
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

        if self._maybe_start_person_follow():
            return

        self._update_smoothed_target()

        if self.time_in_state() >= self.current_search_waypoint.hold_time:
            self._advance_search_or_finish()

    def handle_follow_person(self):
        """Run a simple scripted person response sequence."""

        if self.vehicle_local_position is None:
            return

        if self.person_action_phase == 'STOP_BEFORE_APPROACH':
            # Freeze first so the bbox we memorize comes from a stable hover.
            self._hold_current_position_target()
            if (
                self.person_action_phase_started_at is None
                or self.age_sec(self.person_action_phase_started_at)
                < self.follow_stop_before_approach_sec
            ):
                return

            self.person_action_phase = 'ALIGN_TO_TARGET'
            self.person_action_phase_started_at = self.get_clock().now()
            self.get_logger().info(
                'Person action: aligning yaw to target before forward flight'
            )
            return

        if self.person_action_phase == 'ALIGN_TO_TARGET':
            person_target = self._current_person_target()
            if person_target is not None:
                self.person_align_loss_started_at = None
            else:
                remembered_error = self._get_align_memory_error()
                if remembered_error is None:
                    self._resume_search_after_alignment_loss(
                        'Person action: target lost during alignment, '
                        'resuming patrol'
                    )
                    return
                if self.person_align_loss_started_at is None:
                    self.person_align_loss_started_at = self.get_clock().now()
                elif (
                    self.age_sec(self.person_align_loss_started_at)
                    > self.follow_align_dropout_continue_sec
                ):
                    self._resume_search_after_alignment_loss(
                        'Person action: target was not reacquired during '
                        'alignment, resuming patrol'
                    )
                    return

                self._align_to_person_error(remembered_error)
                return

            if abs(person_target.center_x_error) <= self.follow_yaw_deadband:
                self.get_logger().info(
                    'Person action: target aligned, starting forward flight'
                )
                self._start_bbox_growth_approach()
                return

            self._align_to_person_error(person_target.center_x_error)
            return

        if self.person_action_phase == 'APPROACH_FORWARD':
            person_target = self._current_person_target()
            if person_target is not None:
                self.person_approach_loss_started_at = None
                self.last_approach_person_target = person_target
                self.last_approach_person_target_at = self.get_clock().now()

            if self._has_approach_forward_timed_out():
                self._finish_bbox_growth_approach(
                    'Person action: forward approach timeout, '
                    'holding position'
                )
                return

            if (
                person_target is not None
                and self._has_reached_bbox_growth_target(person_target)
            ):
                self._finish_bbox_growth_approach(
                    'Person action: bbox reached target size, '
                    'holding position'
                )
                return

            if (
                person_target is not None
                and self._is_target_near_bottom_edge(person_target)
            ):
                self._finish_bbox_growth_approach(
                    'Person action: target reached lower camera edge, '
                    'holding position'
                )
                return

            if (
                person_target is None
                and self._should_stop_after_approach_loss()
            ):
                self._finish_bbox_growth_approach(
                    'Person action: target lost during forward approach, '
                    'holding position'
                )
                return

            self._step_forward_for_bbox_growth()
            return

        if self.person_action_phase == 'HOLD_AFTER_APPROACH':
            self._hold_person_follow_position()
            if (
                self.person_action_phase_started_at is None
                or self.age_sec(self.person_action_phase_started_at)
                < self.follow_hover_after_approach_sec
            ):
                return

            self._finish_person_action()
            return

        self._hold_current_position_target()

    def handle_return_home(self):
        """Fly back home, then either resume search or land."""

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
            if self._advance_evacuation_route():
                return
            if self.pending_search_resume_after_home:
                self._resume_search_after_home_reached()
                return
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
            MissionState.FOLLOW_PERSON,
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

    def _maybe_start_person_follow(self) -> bool:
        """Handle target confirmation or switch into person follow."""

        if self.state not in {MissionState.SEARCH, MissionState.HOLD}:
            return False
        if not self.enable_person_follow:
            return False
        if self._is_person_detection_ignored():
            return False

        person_target = self._current_person_target()
        if person_target is None:
            now = self.get_clock().now()
            if self.person_detection_started_at is None:
                self.person_detection_lost_at = None
                return False
            if self.person_detection_lost_at is None:
                self.person_detection_lost_at = now
                return False
            if (
                self.age_sec(self.person_detection_lost_at)
                <= self.follow_detection_gap_tolerance_sec
            ):
                return False

            # A long enough dropout resets confirmation and requires a new
            # continuous detection window before re-engaging.
            self.person_detection_started_at = None
            self.person_detection_lost_at = None
            self.person_follow_confirmation_logged = False
            self.person_reference_bbox_height = 0.0
            return False

        now = self.get_clock().now()
        if self.person_detection_started_at is None:
            self.person_detection_started_at = now
            self.person_follow_confirmation_logged = False
        self.person_detection_lost_at = None

        confirmed_duration = self.age_sec(self.person_detection_started_at)
        if confirmed_duration < self.follow_detection_confirm_sec:
            remaining_sec = max(
                self.follow_detection_confirm_sec - confirmed_duration,
                0.0,
            )
            if not self.person_follow_confirmation_logged:
                self.get_logger().info(
                    'Person detected, confirming target lock for %.1fs '
                    'before visual approach'
                    % self.follow_detection_confirm_sec
                )
                self.person_follow_confirmation_logged = True
            self._hold_current_position_target()
            if self.follow_log_period_sec > 0.0 and (
                self.age_sec(self.last_person_follow_log_at)
                >= self.follow_log_period_sec
            ):
                self.get_logger().info(
                    'Target confirmation in progress: '
                    f'confidence={person_target.confidence:.2f}, '
                    f'remaining={remaining_sec:.1f}s'
                )
                self.last_person_follow_log_at = now
            return True

        self.active_waypoint_started_at = self.get_clock().now()
        self.person_detection_started_at = None
        self.person_follow_confirmation_logged = False
        if self.current_search_waypoint_index is None:
            if self.mission_plan.search_waypoints:
                self.resume_search_waypoint_index = 0
        else:
            self.resume_search_waypoint_index = (
                self.current_search_waypoint_index
            )
        self.set_state(
            MissionState.FOLLOW_PERSON,
            'Person confirmed, stopping search and starting visual approach',
        )
        # Remember the bbox size at the moment of confirmed detection. During
        # the forward segment we compare new detections against this baseline.
        self.person_reference_bbox_height = max(
            float(person_target.height),
            1.0,
        )
        self.get_logger().info(
            'Person follow engaged: '
            f'confidence={person_target.confidence:.2f}, '
            f'center=({person_target.center_x}, {person_target.center_y}), '
            f'bbox_height={person_target.height}, '
            f'height_ratio={person_target.height_ratio:.3f}, '
            f'x_error={self.filtered_person_center_x_error:.3f}'
        )
        self._hold_current_position_target()
        return True

    def _finish_person_action(self):
        """Finish the person interaction and return home before patrol."""

        self.get_logger().info(
            'Person action complete: preparing evacuation return route'
        )
        self.pending_search_resume_after_home = True
        self.pending_teleport_after_home = True
        self._prepare_return_home_waypoint()
        self._prepare_evacuation_route()
        self.set_state(
            MissionState.RETURN_HOME,
            'Person fixed, returning home before resuming search',
        )

    def _resume_search_after_home_reached(self):
        """Teleport the configured model down and continue the search."""

        self._hold_current_position_target()
        self._begin_post_rescue_cooldown()
        if (
            self.pending_teleport_after_home
            and not self.rescue_model_teleported
        ):
            self.rescue_model_teleported = (
                self._teleport_rescue_model_down()
            )
        self.pending_teleport_after_home = False
        self.pending_search_resume_after_home = False
        self.return_home_waypoint = None

        if not self.mission_plan.search_waypoints:
            self._complete_mission()
            return

        if self.resume_search_waypoint_index is None:
            resume_index = 0
        else:
            resume_index = min(
                self.resume_search_waypoint_index,
                len(self.mission_plan.search_waypoints) - 1,
            )
        self._select_search_waypoint(resume_index)
        if self.current_search_waypoint is not None:
            self.target = self._target_from_waypoint(
                self.current_search_waypoint
            )
        self.set_state(
            MissionState.SEARCH,
            'Home reached, resuming search mission',
        )

    def _begin_post_rescue_cooldown(self):
        """Clear tracked target state and ignore detections briefly."""

        self.person_target = None
        self.last_stable_person_target = None
        self.last_stable_person_target_at = None
        self.person_visible_since = None
        self.person_detection_started_at = None
        self.person_detection_lost_at = None
        self.last_person_detection_log_at = None
        self.person_follow_confirmation_logged = False
        if self.post_rescue_detection_cooldown_sec > 0.0:
            self.person_detection_ignore_until = self.get_clock().now()
        else:
            self.person_detection_ignore_until = None

    def _teleport_rescue_model_down(self):
        """Teleport one configured Gazebo model below the scene."""

        if not self.rescue_model_name:
            return

        request = (
            f'name: "{self.rescue_model_name}" '
            f'position {{ '
            f'x: {self.rescue_model_teleport_x} '
            f'y: {self.rescue_model_teleport_y} '
            f'z: {self.rescue_model_teleport_z} }} '
            'orientation { x: 0 y: 0 z: 0 w: 1 }'
        )
        world_candidates = []
        if self.teleport_world_name:
            world_candidates.append(self.teleport_world_name)
        for world_name in ('forest', 'default', 'empty'):
            if world_name not in world_candidates:
                world_candidates.append(world_name)

        for world_name in world_candidates:
            if self._teleport_model_via_service(world_name, request):
                self.get_logger().info(
                    f'Model {self.rescue_model_name} teleported in world '
                    f'{world_name} to z={self.rescue_model_teleport_z:.1f}'
                )
                return True
            if self._teleport_model_via_topic(world_name, request):
                self.get_logger().info(
                    f'Model {self.rescue_model_name} teleported via topic '
                    f'in world {world_name} to '
                    f'z={self.rescue_model_teleport_z:.1f}'
                )
                return True

        self.get_logger().warning(
            f'Could not teleport model {self.rescue_model_name} in any world'
        )
        return False

    def _teleport_model_via_service(
        self,
        world_name: str,
        request: str,
    ) -> bool:
        """Try teleporting a model using Gazebo's set_pose service."""

        command = [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/set_pose',
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(self.rescue_service_timeout_ms),
            '--req',
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(self.rescue_service_timeout_ms / 1000.0, 1.0),
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(
                f'Failed to teleport {self.rescue_model_name}: {error}'
            )
            return False

        return result.returncode == 0

    def _teleport_model_via_topic(
        self,
        world_name: str,
        request: str,
    ) -> bool:
        """Fallback teleport using Gazebo's pose-modify topic."""

        command = [
            'gz',
            'topic',
            '-t',
            f'/world/{world_name}/pose/modify',
            '-m',
            'gz.msgs.Pose',
            '-p',
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(self.rescue_service_timeout_ms / 1000.0, 1.0),
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

        return result.returncode == 0

    def _is_person_detection_ignored(self) -> bool:
        """Return whether fresh detections should be ignored temporarily."""

        if self.person_detection_ignore_until is None:
            return False
        if (
            self.age_sec(self.person_detection_ignore_until)
            >= self.post_rescue_detection_cooldown_sec
        ):
            self.person_detection_ignore_until = None
            return False
        return True

    def _current_person_target(self) -> PersonTarget | None:
        """Return a fresh person target or None when it is stale."""

        if self._is_person_detection_ignored():
            return None
        if self.person_target is None:
            return None
        if self.person_target.confidence < self.follow_min_confidence:
            return None
        if (
            self.last_vision_received_at is None
            or self.age_sec(self.last_vision_received_at)
            > self.follow_person_timeout_sec
        ):
            return None
        return self.person_target

    def _start_bbox_growth_approach(self):
        """Start flying forward until the detected bbox grows enough."""
        if self.person_reference_bbox_height <= 0.0:
            person_target = self._current_person_target()
            if person_target is None:
                self._hold_current_position_target()
                return
            self.person_reference_bbox_height = max(
                float(person_target.height),
                1.0,
            )

        self.person_action_phase = 'APPROACH_FORWARD'
        self.person_action_phase_started_at = self.get_clock().now()
        self.get_logger().info(
            'Person action: flying forward until bbox height grows to %.2fx '
            '(start_height=%d px)'
            % (
                self.follow_bbox_growth_target,
                int(round(self.person_reference_bbox_height)),
            )
        )

    def _resume_search_after_alignment_loss(self, reason: str):
        """Return to patrol when the target cannot be reacquired in align."""

        if self.current_search_waypoint is not None:
            self.active_waypoint_started_at = self.get_clock().now()
            self.set_state(MissionState.SEARCH, reason)
            return

        self._hold_current_position_target()
        self.set_state(MissionState.FAILSAFE, reason)

    def _has_reached_bbox_growth_target(
        self,
        person_target: PersonTarget,
    ) -> bool:
        """Return whether the tracked bbox has grown enough."""

        if self.person_reference_bbox_height <= 0.0:
            return False
        growth_target = max(self.follow_bbox_growth_target, 1.0)
        # Using bbox growth keeps the behavior simple: fly forward until the
        # person appears roughly twice as large as at confirmed detection time.
        return (
            float(person_target.height)
            >= self.person_reference_bbox_height * growth_target
        )

    def _is_target_near_bottom_edge(self, person_target: PersonTarget) -> bool:
        """Return whether the bbox is about to leave through frame bottom."""

        edge_margin = min(
            max(self.follow_bottom_edge_margin_ratio, 0.0),
            0.5,
        )
        return person_target.bottom_ratio >= 1.0 - edge_margin

    def _should_stop_after_approach_loss(self) -> bool:
        """Stop forward approach when the target is not safely tracked."""

        now = self.get_clock().now()
        if self.person_approach_loss_started_at is None:
            self.person_approach_loss_started_at = now

        if (
            self.last_approach_person_target is not None
            and self._is_target_near_bottom_edge(
                self.last_approach_person_target
            )
        ):
            return True

        return (
            self.age_sec(self.person_approach_loss_started_at)
            >= max(self.follow_approach_dropout_stop_sec, 0.0)
        )

    def _has_approach_forward_timed_out(self) -> bool:
        """Return whether the forward approach has taken too long."""

        if self.follow_approach_max_sec <= 0.0:
            return False
        if self.person_action_phase_started_at is None:
            return False
        return (
            self.age_sec(self.person_action_phase_started_at)
            >= self.follow_approach_max_sec
        )

    def _finish_bbox_growth_approach(self, reason: str):
        """Stop forward motion and transition to the post-approach hover."""

        self.person_action_phase = 'HOLD_AFTER_APPROACH'
        self.person_action_phase_started_at = self.get_clock().now()
        self.person_approach_loss_started_at = None
        self._hold_person_follow_position()
        self.get_logger().info(reason)

    def _step_forward_for_bbox_growth(self):
        """Advance the target forward with a small lead so PX4 keeps moving."""

        if self.vehicle_local_position is None:
            return

        heading = self.vehicle_local_position.heading
        if not math.isfinite(heading):
            heading = self.target.yaw
        if not math.isfinite(heading):
            heading = 0.0

        step_distance = (
            max(self.follow_forward_speed, 0.0) * self.control_period
        )
        lead_dx = self.target.x - self.vehicle_local_position.x
        lead_dy = self.target.y - self.vehicle_local_position.y
        lead_distance = math.hypot(lead_dx, lead_dy)
        if lead_distance > self.follow_target_lead_limit:
            base_x = self.vehicle_local_position.x
            base_y = self.vehicle_local_position.y
        else:
            base_x = self.target.x
            base_y = self.target.y

        self.target = TargetState(
            x=base_x + math.cos(heading) * step_distance,
            y=base_y + math.sin(heading) * step_distance,
            z=self.vehicle_local_position.z,
            yaw=heading,
        )

    def _get_align_memory_error(self) -> float | None:
        """Return the last usable horizontal error for short align recovery."""

        if (
            self.follow_target_memory_timeout_sec <= 0.0
            or self.last_stable_person_target is None
            or self.last_stable_person_target_at is None
            or self.age_sec(self.last_stable_person_target_at)
            > self.follow_target_memory_timeout_sec
        ):
            if abs(self.last_significant_person_center_x_error) <= (
                self.follow_yaw_deadband
            ):
                return None
            return self.last_significant_person_center_x_error

        remembered_error = self.last_stable_person_target.center_x_error
        if abs(self.last_significant_person_center_x_error) > (
            self.follow_yaw_deadband
        ):
            remembered_error = self.last_significant_person_center_x_error
        if abs(remembered_error) <= self.follow_yaw_deadband:
            return None
        return remembered_error

    def _align_to_person_error(self, yaw_error: float):
        """Rotate in place toward a live or remembered horizontal error."""

        if self.vehicle_local_position is None:
            return

        self.person_yaw_locked = False
        self.person_hold_yaw = None
        self.person_centered_since = None

        heading = self.vehicle_local_position.heading
        if not math.isfinite(heading):
            heading = self.target.yaw
        if not math.isfinite(heading):
            heading = 0.0

        yaw_rate_cmd = self.follow_yaw_kp * yaw_error
        yaw_rate_cmd = max(
            min(yaw_rate_cmd, self.max_yaw_rate),
            -self.max_yaw_rate,
        )
        next_yaw = self._normalize_angle(
            heading + yaw_rate_cmd * self.control_period
        )

        self.filtered_person_center_x_error = yaw_error
        self.previous_person_center_x_error = yaw_error
        self.filtered_follow_yaw_rate = yaw_rate_cmd
        self.previous_yaw_error_sample_at = self.get_clock().now()

        self.target = TargetState(
            x=self.vehicle_local_position.x,
            y=self.vehicle_local_position.y,
            z=self.vehicle_local_position.z,
            yaw=next_yaw,
        )

    def _should_stop_near_person(self, person_target: PersonTarget) -> bool:
        """Stop advancing when the target appears close enough."""

        return bool(
            person_target.height_ratio >= self.follow_stop_bbox_height_ratio
        )

    def _is_person_follow_centered(self) -> bool:
        """Return whether yaw should stay locked on the centered target."""

        error = abs(self.filtered_person_center_x_error)
        now = self.get_clock().now()

        if self.person_yaw_locked:
            if error > self.follow_yaw_unlock_deadband:
                self.person_yaw_locked = False
                self.person_hold_yaw = None
                self.person_centered_since = None
                return False
            return True

        if error > self.follow_yaw_deadband:
            self.person_centered_since = None
            return False

        if self.person_centered_since is None:
            self.person_centered_since = now
            return False

        if (
            self.age_sec(self.person_centered_since)
            < self.follow_centered_hold_sec
        ):
            return False

        if self.target is not None and math.isfinite(self.target.yaw):
            self.person_hold_yaw = self.target.yaw
        else:
            self.person_hold_yaw = self.vehicle_local_position.heading
        self.person_yaw_locked = True
        self.filtered_follow_yaw_rate = 0.0
        return True

    def _hold_person_follow_position(self):
        """Freeze position while the target remains centered."""

        if self.vehicle_local_position is None:
            return

        hold_yaw = self.person_hold_yaw
        if hold_yaw is None or not math.isfinite(hold_yaw):
            hold_yaw = self.vehicle_local_position.heading
        if not math.isfinite(hold_yaw):
            hold_yaw = self.target.yaw

        self.target = TargetState(
            x=self.vehicle_local_position.x,
            y=self.vehicle_local_position.y,
            z=self.vehicle_local_position.z,
            yaw=hold_yaw,
        )
        self.previous_person_center_x_error = (
            self.filtered_person_center_x_error
        )
        self.filtered_follow_yaw_rate = 0.0
        self.previous_yaw_error_sample_at = self.get_clock().now()

    def _is_person_follow_alignment_phase(
        self,
        person_target: PersonTarget,
    ) -> bool:
        """Return whether follow mode should still spend time aligning."""

        if self.person_follow_started_at is None:
            return False
        if self.follow_alignment_hold_sec <= 0.0:
            return False

        if self.person_yaw_locked:
            return False

        alignment_error = abs(self.filtered_person_center_x_error)
        if (
            person_target.left_ratio <= self.follow_frame_edge_margin_ratio
            or person_target.right_ratio
            >= 1.0 - self.follow_frame_edge_margin_ratio
        ):
            self.person_centered_since = None
            return True

        if alignment_error > self.follow_yaw_unlock_deadband:
            self.person_centered_since = None
            return True

        if self.person_centered_since is None:
            self.person_centered_since = self.get_clock().now()

        return (
            self.age_sec(self.person_follow_started_at)
            < self.follow_alignment_hold_sec
            or self.age_sec(self.person_centered_since)
            < self.follow_centered_hold_sec
        )

    def _update_person_follow_target(
        self,
        person_target: PersonTarget,
        allow_forward_motion: bool,
    ):
        """Shift the position target toward the detected person."""

        heading = self.vehicle_local_position.heading
        if not math.isfinite(heading):
            heading = self.target.yaw

        if self.person_yaw_locked:
            self._update_locked_person_follow_target(
                person_target,
                allow_forward_motion=allow_forward_motion,
            )
            return

        lead_dx = self.target.x - self.vehicle_local_position.x
        lead_dy = self.target.y - self.vehicle_local_position.y
        lead_distance = math.hypot(lead_dx, lead_dy)
        if lead_distance > self.follow_target_lead_limit:
            base_x = self.vehicle_local_position.x
            base_y = self.vehicle_local_position.y
            base_yaw = heading
        else:
            base_x = self.target.x
            base_y = self.target.y
            if math.isfinite(self.target.yaw):
                base_yaw = self.target.yaw
            else:
                base_yaw = heading

        horizontal_error = self.filtered_person_center_x_error
        yaw_rate_cmd = 0.0
        if abs(horizontal_error) > self.follow_yaw_deadband:
            now = self.get_clock().now()
            derivative = 0.0
            if self.previous_yaw_error_sample_at is not None:
                dt = self.age_sec(self.previous_yaw_error_sample_at)
                if dt > 1e-3:
                    derivative = (
                        horizontal_error - self.previous_person_center_x_error
                    ) / dt
            yaw_rate_cmd = (
                self.follow_yaw_kp * horizontal_error
                + self.follow_yaw_kd * derivative
            )
            self.previous_person_center_x_error = horizontal_error
            self.previous_yaw_error_sample_at = now
        else:
            self.previous_person_center_x_error = horizontal_error
            self.previous_yaw_error_sample_at = self.get_clock().now()

        yaw_rate_cmd = max(
            min(yaw_rate_cmd, self.max_yaw_rate),
            -self.max_yaw_rate,
        )
        yaw_rate_alpha = min(
            max(self.follow_yaw_rate_filter_alpha, 0.0),
            1.0,
        )
        self.filtered_follow_yaw_rate = (
            yaw_rate_alpha * yaw_rate_cmd
            + (1.0 - yaw_rate_alpha) * self.filtered_follow_yaw_rate
        )
        next_yaw = self._normalize_angle(
            base_yaw + self.filtered_follow_yaw_rate * self.control_period
        )

        forward_step = 0.0
        if allow_forward_motion:
            forward_speed = self.follow_forward_speed
            if (
                person_target.height_ratio
                >= self.follow_slow_bbox_height_ratio
            ):
                forward_speed = min(
                    forward_speed,
                    self.follow_slow_forward_speed,
                )
            forward_step = max(forward_speed, 0.0) * self.control_period

        lateral_step = 0.0
        if (
            allow_forward_motion
            and abs(horizontal_error) > self.follow_lateral_deadband
        ):
            lateral_step = (
                -horizontal_error
                * self.follow_lateral_gain
                * self.control_period
            )

        self.target = TargetState(
            x=base_x
            + math.cos(next_yaw) * forward_step
            - math.sin(next_yaw) * lateral_step,
            y=base_y
            + math.sin(next_yaw) * forward_step
            + math.cos(next_yaw) * lateral_step,
            z=self.mission_plan.takeoff_waypoint.z,
            yaw=next_yaw,
        )

    def _update_locked_person_follow_target(
        self,
        person_target: PersonTarget,
        allow_forward_motion: bool,
    ):
        """Advance straight ahead while keeping the previously locked yaw."""

        if self.vehicle_local_position is None:
            return

        locked_yaw = self.person_hold_yaw
        if locked_yaw is None or not math.isfinite(locked_yaw):
            locked_yaw = self.target.yaw
        if not math.isfinite(locked_yaw):
            locked_yaw = self.vehicle_local_position.heading

        base_x = self.vehicle_local_position.x
        base_y = self.vehicle_local_position.y

        forward_step = 0.0
        if allow_forward_motion:
            forward_speed = self.follow_forward_speed
            if (
                person_target.height_ratio
                >= self.follow_slow_bbox_height_ratio
            ):
                forward_speed = min(
                    forward_speed,
                    self.follow_slow_forward_speed,
                )
            forward_step = max(forward_speed, 0.0) * self.control_period

        self.target = TargetState(
            x=base_x + math.cos(locked_yaw) * forward_step,
            y=base_y + math.sin(locked_yaw) * forward_step,
            z=self.mission_plan.takeoff_waypoint.z,
            yaw=locked_yaw,
        )

    def _log_person_follow_status(
        self,
        person_target: PersonTarget,
        stopped: bool,
        aligning: bool = False,
    ):
        """Throttle follow logs so approach behavior is easy to inspect."""

        if self.follow_log_period_sec <= 0.0:
            return
        if (
            self.age_sec(self.last_person_follow_log_at)
            < self.follow_log_period_sec
        ):
            return

        if stopped:
            action = 'holding near person'
        elif aligning:
            action = 'aligning before approach'
        else:
            action = 'approaching person'
        self.get_logger().info(
            f'{action}: '
            f'confidence={person_target.confidence:.2f}, '
            f'height_ratio={person_target.height_ratio:.2f}, '
            f'center_x_error={self.filtered_person_center_x_error:.2f}'
        )
        self.last_person_follow_log_at = self.get_clock().now()

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to the [-pi, pi] interval."""

        return math.atan2(math.sin(angle), math.cos(angle))

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

        self.evacuation_route_waypoints = []
        self.evacuation_route_index = 0

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

    def _prepare_evacuation_route(self):
        """Plan an obstacle-aware route from current pose to home."""

        if not self.enable_evacuation_astar:
            return
        if self.vehicle_local_position is None:
            return
        if self.home_position is None or self.return_home_waypoint is None:
            return

        start = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
        )
        goal = (
            float(self.return_home_waypoint.x),
            float(self.return_home_waypoint.y),
        )
        config = PlannerConfig(
            world_sdf_path=self.evacuation_world_sdf_path,
            grid_resolution=self.evacuation_grid_resolution,
            obstacle_margin=self.evacuation_obstacle_margin,
            map_padding=self.evacuation_map_padding,
            waypoint_spacing=self.evacuation_waypoint_spacing,
            mission_rotation_deg=self.evacuation_mission_rotation_deg,
            mission_invert_y=self.evacuation_mission_invert_y,
            max_iterations=self.evacuation_max_iterations,
        )
        planner = EvacuationRoutePlanner(config)
        route_xy = planner.plan(start=start, goal=goal)
        if len(route_xy) < 2:
            self.get_logger().warning(
                'A* evacuation route was not found, using direct return-home'
            )
            return

        return_yaw = self.return_home_waypoint.yaw
        altitude = self.mission_plan.takeoff_waypoint.z
        # Skip the first point because it is the current vehicle position.
        self.evacuation_route_waypoints = [
            Waypoint(x=x, y=y, z=altitude, yaw=return_yaw)
            for x, y in route_xy[1:]
        ]
        self.evacuation_route_index = 0
        self.return_home_waypoint = self.evacuation_route_waypoints[0]
        self.active_waypoint_started_at = self.get_clock().now()
        self._publish_evacuation_route(route_xy)
        self.get_logger().info(
            'A* evacuation route planned: '
            f'obstacles={len(planner.obstacles)}, '
            f'waypoints={len(self.evacuation_route_waypoints)}'
        )
        self._log_evacuation_waypoint()

    def _publish_evacuation_route(self, route_xy: list[tuple[float, float]]):
        """Publish the latest A* route for map viewers."""

        message = Float32MultiArray()
        data: list[float] = []
        for x, y in route_xy:
            data.extend([float(x), float(y)])
        message.data = data
        self.evacuation_route_pub.publish(message)

    def _advance_evacuation_route(self) -> bool:
        """Advance to the next A* evacuation waypoint if one exists."""

        if not self.evacuation_route_waypoints:
            return False
        if self.evacuation_route_index >= len(
            self.evacuation_route_waypoints
        ) - 1:
            self.evacuation_route_waypoints = []
            self.evacuation_route_index = 0
            return False

        self.evacuation_route_index += 1
        self.return_home_waypoint = self.evacuation_route_waypoints[
            self.evacuation_route_index
        ]
        self.active_waypoint_started_at = self.get_clock().now()
        self._log_evacuation_waypoint()
        return True

    def _log_evacuation_waypoint(self):
        """Log active A* evacuation waypoint."""

        if self.return_home_waypoint is None:
            return
        if not self.evacuation_route_waypoints:
            return

        self.get_logger().info(
            'A* evacuation waypoint %d/%d -> x=%.2f y=%.2f z=%.2f'
            % (
                self.evacuation_route_index + 1,
                len(self.evacuation_route_waypoints),
                self.return_home_waypoint.x,
                self.return_home_waypoint.y,
                self.return_home_waypoint.z,
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
