"""Mission data structures for the PX4 offboard waypoint mission."""

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
class MissionPlan:
    """Mission definition kept independent from ROS2/PX4 runtime logic."""

    takeoff_waypoint: Waypoint
    search_waypoints: tuple[Waypoint, ...]


def default_mission_plan(takeoff_height: float) -> MissionPlan:
    """Build the default mission while preserving the existing route."""

    working_altitude = -abs(takeoff_height)

    return MissionPlan(
        takeoff_waypoint=Waypoint(x=0.0, y=0.0, z=working_altitude, yaw=0.0),
        search_waypoints=(
            Waypoint(x=16.0, y=0.0, z=working_altitude, yaw=0.0),
            Waypoint(
                x=16.0,
                y=4.0,
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=1.5,
            ),
            Waypoint(
                x=0.0,
                y=4.0,
                z=working_altitude,
                yaw=math.pi,
                hold_time=1.5,
            ),
            Waypoint(
                x=0.0,
                y=8.0,
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=1.5,
            ),
            Waypoint(
                x=16.0,
                y=8.0,
                z=working_altitude,
                yaw=0.0,
                hold_time=1.5,
            ),
            Waypoint(
                x=16.0,
                y=12.0,
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=1.5,
            ),
            Waypoint(
                x=0.0,
                y=12.0,
                z=working_altitude,
                yaw=math.pi,
                hold_time=1.5,
            ),
            Waypoint(x=0.0, y=0.0, z=working_altitude, yaw=-math.pi / 2.0),
        ),
    )
