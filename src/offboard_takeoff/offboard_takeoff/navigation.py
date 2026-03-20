"""Small helpers for waypoint navigation and target smoothing."""

from dataclasses import dataclass
import math

from offboard_takeoff.mission import MissionConfig
from offboard_takeoff.mission import Waypoint


@dataclass
class TargetState:
    """Commanded local position and yaw sent to PX4."""

    x: float
    y: float
    z: float
    yaw: float


def wrap_to_pi(angle: float) -> float:
    """Normalize angle to the [-pi, pi] interval."""

    return math.atan2(math.sin(angle), math.cos(angle))


def distance_to_waypoint(
    current_x: float,
    current_y: float,
    current_z: float,
    waypoint: Waypoint,
) -> float:
    """Compute Euclidean distance to a waypoint."""

    dx = waypoint.x - current_x
    dy = waypoint.y - current_y
    dz = waypoint.z - current_z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def yaw_error_to_waypoint(current_yaw: float, waypoint: Waypoint) -> float:
    """Compute shortest yaw error to a waypoint heading."""

    return wrap_to_pi(waypoint.yaw - current_yaw)


def has_reached_waypoint(
    current_x: float,
    current_y: float,
    current_z: float,
    current_yaw: float,
    waypoint: Waypoint,
    config: MissionConfig,
) -> bool:
    """Check whether current pose is close enough to the active waypoint."""

    distance = distance_to_waypoint(current_x, current_y, current_z, waypoint)
    yaw_error = yaw_error_to_waypoint(current_yaw, waypoint)
    return (
        distance <= config.position_tolerance
        and abs(yaw_error) <= config.yaw_tolerance
    )


def step_towards(current: float, goal: float, max_delta: float) -> float:
    """Move one scalar value toward its goal with limited step size."""

    delta = goal - current
    if abs(delta) <= max_delta:
        return goal
    return current + math.copysign(max_delta, delta)


def smooth_target_towards_waypoint(
    target: TargetState,
    waypoint: Waypoint,
    config: MissionConfig,
) -> TargetState:
    """Advance the commanded target toward the waypoint with limited speed."""

    max_position_step = config.max_speed * config.control_period

    dx = waypoint.x - target.x
    dy = waypoint.y - target.y
    dz = waypoint.z - target.z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    if distance > 0.0:
        position_step = min(distance, max_position_step)
        scale = position_step / distance
        target.x += dx * scale
        target.y += dy * scale
        target.z += dz * scale

    max_yaw_step = config.max_yaw_rate * config.control_period
    yaw_error = wrap_to_pi(waypoint.yaw - target.yaw)
    target.yaw = wrap_to_pi(
        step_towards(target.yaw, target.yaw + yaw_error, max_yaw_step)
    )

    return target
