"""Mission configuration for the PX4 offboard waypoint example."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Waypoint:
    """Single waypoint in PX4 local NED coordinates."""

    x: float
    y: float
    z: float
    yaw: float
    hold_time: float = 0.0


@dataclass(frozen=True)
class MissionConfig:
    """Tunable mission parameters."""

    waypoints: list[Waypoint]
    position_tolerance: float = 0.4
    yaw_tolerance: float = 0.3
    max_speed: float = 0.8
    max_yaw_rate: float = 0.4
    control_period: float = 0.1


def default_mission_config() -> MissionConfig:
    """Build the default waypoint mission."""

    return MissionConfig(
        waypoints=[
            Waypoint(x=0.0, y=0.0, z=-3.0, yaw=0.0),
            Waypoint(x=16.0, y=0.0, z=-3.0, yaw=0.0),
            Waypoint(x=16.0, y=4.0, z=-3.0, yaw=math.pi / 2.0, hold_time=1.5),
            Waypoint(x=0.0, y=4.0, z=-3.0, yaw=math.pi, hold_time=1.5),
            Waypoint(x=0.0, y=8.0, z=-3.0, yaw=math.pi / 2.0, hold_time=1.5),
            Waypoint(x=16.0, y=8.0, z=-3.0, yaw=0.0, hold_time=1.5),
            Waypoint(x=16.0, y=12.0, z=-3.0, yaw=math.pi / 2.0, hold_time=1.5),
            Waypoint(x=0.0, y=12.0, z=-3.0, yaw=math.pi, hold_time=1.5),
            Waypoint(x=0.0, y=0.0, z=-3.0, yaw=-math.pi / 2.0),
        ]
    )
