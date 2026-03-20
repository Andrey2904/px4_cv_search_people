"""Helpers for waypoint navigation and setpoint smoothing."""

from dataclasses import dataclass
import math

from offboard_takeoff.mission import Waypoint


@dataclass
class TargetState:
    """Commanded local position and yaw sent to PX4."""

    x: float
    y: float
    z: float
    yaw: float


def normalize_angle(angle: float) -> float:
    """Normalize angle to the [-pi, pi] interval."""

    return math.atan2(math.sin(angle), math.cos(angle))


def is_position_reached(
    current_x: float,
    current_y: float,
    current_z: float,
    waypoint: Waypoint,
    tolerance: float,
) -> bool:
    """Check whether current position is close enough to a waypoint."""

    dx = waypoint.x - current_x
    dy = waypoint.y - current_y
    dz = waypoint.z - current_z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    return distance <= tolerance


def is_yaw_reached(
    current_yaw: float,
    target_yaw: float,
    tolerance: float,
) -> bool:
    """Check whether current yaw is close enough to the target yaw."""

    return abs(normalize_angle(target_yaw - current_yaw)) <= tolerance


def limit_vector_step(
    dx: float,
    dy: float,
    dz: float,
    max_step: float,
) -> tuple[float, float, float]:
    """Limit a 3D step without changing its direction."""

    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance == 0.0 or distance <= max_step:
        return dx, dy, dz

    scale = max_step / distance
    return dx * scale, dy * scale, dz * scale


def limit_yaw_rate(yaw_error: float, max_step: float) -> float:
    """Limit yaw change per control step."""

    if abs(yaw_error) <= max_step:
        return yaw_error
    return math.copysign(max_step, yaw_error)


def smooth_target_towards_waypoint(
    target: TargetState,
    waypoint: Waypoint,
    control_period: float,
    max_speed: float,
    max_yaw_rate: float,
) -> TargetState:
    """Advance the commanded target toward the waypoint with limited speed."""

    max_position_step = max_speed * control_period
    step_x, step_y, step_z = limit_vector_step(
        waypoint.x - target.x,
        waypoint.y - target.y,
        waypoint.z - target.z,
        max_position_step,
    )

    yaw_error = normalize_angle(waypoint.yaw - target.yaw)
    yaw_step = limit_yaw_rate(yaw_error, max_yaw_rate * control_period)

    return TargetState(
        x=target.x + step_x,
        y=target.y + step_y,
        z=target.z + step_z,
        yaw=normalize_angle(target.yaw + yaw_step),
    )
