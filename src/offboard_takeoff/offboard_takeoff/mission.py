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
    """Build the default mission with a wider snake-style search route."""

    working_altitude = -abs(takeoff_height)
    search_min_x = 0.0
    search_max_x = 24.0
    search_rows_y = (0.0, 7.0, 14.0, 21.0, 28.0, 20.0, 24.0)
    row_hold_sec = 1.5

    return MissionPlan(
        takeoff_waypoint=Waypoint(x=0.0, y=0.0, z=working_altitude, yaw=0.0),
        search_waypoints=(
            Waypoint(
                x=search_max_x,
                y=search_rows_y[0],
                z=working_altitude,
                yaw=0.0,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[1],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[1],
                z=working_altitude,
                yaw=math.pi,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[2],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[2],
                z=working_altitude,
                yaw=0.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[3],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[3],
                z=working_altitude,
                yaw=math.pi,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[4],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[4],
                z=working_altitude,
                yaw=0.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[5],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[5],
                z=working_altitude,
                yaw=math.pi,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[6],
                z=working_altitude,
                yaw=math.pi / 2.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_max_x,
                y=search_rows_y[6],
                z=working_altitude,
                yaw=0.0,
                hold_time=row_hold_sec,
            ),
            Waypoint(
                x=search_min_x,
                y=search_rows_y[0],
                z=working_altitude,
                yaw=-math.pi / 2.0,
            ),
        ),
    )
