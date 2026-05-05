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
    """Build the default mission around the forest search area."""

    working_altitude = -abs(takeoff_height)
    # The top-down map uses mission_x ~= Gazebo world Y and
    # mission_y ~= Gazebo world X.  The forest trees occupy approximately
    # Gazebo X -46..25 and Gazebo Y -56..-25, so this route covers the full
    # forest with 7 m spacing between snake passes.
    search_min_x = -60.0
    search_max_x = -20.0
    search_max_y = 30.0
    search_min_y = -50.0
    search_row_spacing = 7.0
    row_hold_sec = 1.5
    search_row_count = int(
        math.floor((search_max_y - search_min_y) / search_row_spacing)
    ) + 1
    search_rows_y = tuple(
        search_max_y - index * search_row_spacing
        for index in range(search_row_count)
    )
    if search_rows_y[-1] > search_min_y:
        search_rows_y = (*search_rows_y, search_min_y)

    search_waypoints: list[Waypoint] = []
    current_x = search_max_x
    for index, row_y in enumerate(search_rows_y):
        target_x = search_min_x if index % 2 == 0 else search_max_x
        row_yaw = math.pi if target_x < current_x else 0.0

        if index == 0:
            search_waypoints.append(
                Waypoint(
                    x=current_x,
                    y=row_y,
                    z=working_altitude,
                    yaw=row_yaw,
                )
            )

        search_waypoints.append(
            Waypoint(
                x=target_x,
                y=row_y,
                z=working_altitude,
                yaw=row_yaw,
                hold_time=row_hold_sec,
            )
        )
        current_x = target_x

        if index + 1 < len(search_rows_y):
            next_row_y = search_rows_y[index + 1]
            turn_yaw = -math.pi / 2.0 if next_row_y < row_y else math.pi / 2.0
            search_waypoints.append(
                Waypoint(
                    x=current_x,
                    y=next_row_y,
                    z=working_altitude,
                    yaw=turn_yaw,
                    hold_time=row_hold_sec,
                )
            )

    return MissionPlan(
        takeoff_waypoint=Waypoint(x=0.0, y=0.0, z=working_altitude, yaw=0.0),
        search_waypoints=tuple(search_waypoints),
    )
