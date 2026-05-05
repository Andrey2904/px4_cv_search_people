"""Grid A* evacuation route planner for the Gazebo SAR scene."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
import re
from xml.etree import ElementTree as ET


@dataclass(frozen=True)
class PlannerConfig:
    """A* planner configuration."""

    world_sdf_path: str
    grid_resolution: float = 1.0
    obstacle_margin: float = 3.0
    map_padding: float = 8.0
    waypoint_spacing: float = 3.0
    mission_rotation_deg: float = 90.0
    mission_invert_y: bool = True
    max_iterations: int = 20000


@dataclass(frozen=True)
class Obstacle:
    """Inflated obstacle polygon in PX4 mission-local XY coordinates."""

    name: str
    corners: tuple[tuple[float, float], ...]


MODEL_FOOTPRINTS = {
    'house': (16.27, 6.07),
    'destroy_building': (30.58, 28.97),
    'roof_rubble': (6.03, 6.03),
    'rubble': (10.79, 4.09),
    'mango_tree': (3.38, 3.38),
}

EXCLUDED_MODEL_NAMES = {
    'grave',
    'man',
    'men',
    'person',
    'people',
    'victim',
    'walker',
}


class EvacuationRoutePlanner:
    """Plan an obstacle-avoiding 2D route from rescue point to evacuation."""

    def __init__(self, config: PlannerConfig):
        self.config = config
        self.obstacles = self._load_obstacles()

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> list[tuple[float, float]]:
        """Plan a route in mission-local XY coordinates."""

        resolution = max(self.config.grid_resolution, 0.2)
        bounds = self._compute_bounds(start, goal)
        min_x, max_x, min_y, max_y = bounds
        cols = int(math.ceil((max_x - min_x) / resolution)) + 1
        rows = int(math.ceil((max_y - min_y) / resolution)) + 1

        occupied = [
            [False for _ in range(cols)]
            for _ in range(rows)
        ]
        for row in range(rows):
            y = min_y + row * resolution
            for col in range(cols):
                x = min_x + col * resolution
                occupied[row][col] = self._is_occupied(x, y)

        start_cell = self._nearest_free_cell(
            self._point_to_cell(start, min_x, min_y, resolution),
            occupied,
        )
        goal_cell = self._nearest_free_cell(
            self._point_to_cell(goal, min_x, min_y, resolution),
            occupied,
        )
        if start_cell is None or goal_cell is None:
            return []

        cell_path = self._astar(start_cell, goal_cell, occupied)
        if not cell_path:
            return []

        raw_points = [
            self._cell_to_point(cell, min_x, min_y, resolution)
            for cell in cell_path
        ]
        smoothed = self._smooth_path(raw_points, occupied, bounds, resolution)
        return self._thin_waypoints(smoothed)

    def _load_obstacles(self) -> list[Obstacle]:
        """Load SDF includes as inflated obstacle polygons."""

        world_path = Path(self.config.world_sdf_path).expanduser()
        if not world_path.is_file():
            return []

        try:
            root = ET.fromstring(self._read_world_sdf_xml(world_path))
        except ET.ParseError:
            return []

        world_node = root.find('world')
        if world_node is None:
            return []

        obstacles: list[Obstacle] = []
        for include_node in world_node.findall('include'):
            uri = self._child_text(include_node, 'uri')
            name = self._child_text(include_node, 'name')
            model_name = self._model_name_from_uri(uri)
            if self._is_excluded(model_name, name):
                continue
            footprint = MODEL_FOOTPRINTS.get(model_name)
            if footprint is None:
                continue

            pose = self._parse_pose(self._child_text(include_node, 'pose'))
            corners = self._inflated_obstacle_corners(
                x=pose[0],
                y=pose[1],
                yaw=pose[5],
                size_x=footprint[0],
                size_y=footprint[1],
            )
            obstacles.append(
                Obstacle(
                    name=name or model_name,
                    corners=tuple(corners),
                )
            )
        return obstacles

    def _read_world_sdf_xml(self, world_path: Path) -> str:
        """Read XML and repair a known malformed comment style."""

        text = world_path.read_text(encoding='utf-8', errors='replace')
        if '<!---' in text and '--->' in text:
            text = text.replace('<!---', '<!--')
            text = text.replace('--->', '-->')
        return text

    def _child_text(self, node: ET.Element, child_name: str) -> str:
        """Return stripped child text or an empty string."""

        child = node.find(child_name)
        if child is None or child.text is None:
            return ''
        return child.text.strip()

    def _model_name_from_uri(self, uri: str) -> str:
        """Extract model name from model:// URI."""

        if not uri:
            return ''
        return (
            uri.split('://', maxsplit=1)[-1]
            .strip('/')
            .split('/')[-1]
            .lower()
        )

    def _is_excluded(self, model_name: str, instance_name: str) -> bool:
        """Return whether a model should not be treated as an obstacle."""

        text = f'{model_name} {instance_name}'.lower().replace('_', ' ')
        tokens = {
            token
            for token in re.split(r'[^a-z0-9]+', text)
            if token
        }
        if model_name in {'grave'}:
            return True
        return bool(tokens & EXCLUDED_MODEL_NAMES)

    def _parse_pose(self, pose_text: str) -> tuple[float, ...]:
        """Parse x y z roll pitch yaw from an SDF pose string."""

        values = [0.0] * 6
        for index, raw_value in enumerate(pose_text.split()[:6]):
            try:
                values[index] = float(raw_value)
            except ValueError:
                values[index] = 0.0
        return tuple(values)

    def _inflated_obstacle_corners(
        self,
        x: float,
        y: float,
        yaw: float,
        size_x: float,
        size_y: float,
    ) -> list[tuple[float, float]]:
        """Create an inflated obstacle polygon in mission coordinates."""

        half_x = size_x / 2.0 + self.config.obstacle_margin
        half_y = size_y / 2.0 + self.config.obstacle_margin
        local_corners = (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        corners = []
        for local_x, local_y in local_corners:
            world_x = x + local_x * cos_yaw - local_y * sin_yaw
            world_y = y + local_x * sin_yaw + local_y * cos_yaw
            corners.append(self._map_to_mission_xy(world_x, world_y))
        return corners

    def _map_to_mission_xy(
        self,
        map_x: float,
        map_y: float,
    ) -> tuple[float, float]:
        """Invert the mission-to-map transform used by the top-down viewer."""

        x = map_x
        y = map_y
        rotation_rad = math.radians(-self.config.mission_rotation_deg)
        rotated_x = x * math.cos(rotation_rad) - y * math.sin(rotation_rad)
        rotated_y = x * math.sin(rotation_rad) + y * math.cos(rotation_rad)

        if self.config.mission_invert_y:
            rotated_y = -rotated_y
        return rotated_x, rotated_y

    def _compute_bounds(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> tuple[float, float, float, float]:
        """Compute grid bounds around route endpoints and obstacles."""

        xs = [start[0], goal[0]]
        ys = [start[1], goal[1]]
        for obstacle in self.obstacles:
            for x, y in obstacle.corners:
                xs.append(x)
                ys.append(y)

        padding = max(self.config.map_padding, self.config.obstacle_margin)
        return (
            min(xs) - padding,
            max(xs) + padding,
            min(ys) - padding,
            max(ys) + padding,
        )

    def _is_occupied(self, x: float, y: float) -> bool:
        """Return whether a point lies inside any obstacle."""

        return any(
            self._point_in_polygon((x, y), obstacle.corners)
            for obstacle in self.obstacles
        )

    def _point_in_polygon(
        self,
        point: tuple[float, float],
        polygon: tuple[tuple[float, float], ...],
    ) -> bool:
        """Ray-casting point-in-polygon test."""

        x, y = point
        inside = False
        count = len(polygon)
        for index in range(count):
            x1, y1 = polygon[index]
            x2, y2 = polygon[(index + 1) % count]
            if (y1 > y) == (y2 > y):
                continue
            intersect_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersect_x:
                inside = not inside
        return inside

    def _point_to_cell(
        self,
        point: tuple[float, float],
        min_x: float,
        min_y: float,
        resolution: float,
    ) -> tuple[int, int]:
        """Convert XY point to grid cell."""

        col = int(round((point[0] - min_x) / resolution))
        row = int(round((point[1] - min_y) / resolution))
        return row, col

    def _cell_to_point(
        self,
        cell: tuple[int, int],
        min_x: float,
        min_y: float,
        resolution: float,
    ) -> tuple[float, float]:
        """Convert grid cell to XY point."""

        row, col = cell
        return min_x + col * resolution, min_y + row * resolution

    def _nearest_free_cell(
        self,
        cell: tuple[int, int],
        occupied: list[list[bool]],
    ) -> tuple[int, int] | None:
        """Find nearest free cell around the requested cell."""

        rows = len(occupied)
        cols = len(occupied[0]) if rows else 0
        row, col = cell
        if 0 <= row < rows and 0 <= col < cols and not occupied[row][col]:
            return row, col

        for radius in range(1, max(rows, cols)):
            for d_row in range(-radius, radius + 1):
                for d_col in range(-radius, radius + 1):
                    if max(abs(d_row), abs(d_col)) != radius:
                        continue
                    n_row = row + d_row
                    n_col = col + d_col
                    if not (0 <= n_row < rows and 0 <= n_col < cols):
                        continue
                    if not occupied[n_row][n_col]:
                        return n_row, n_col
        return None

    def _astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        occupied: list[list[bool]],
    ) -> list[tuple[int, int]]:
        """Run 8-connected grid A*."""

        rows = len(occupied)
        cols = len(occupied[0]) if rows else 0
        open_heap = [(0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        cost_so_far = {start: 0.0}
        iterations = 0
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )

        while open_heap and iterations < self.config.max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for d_row, d_col, step_cost in neighbors:
                next_cell = current[0] + d_row, current[1] + d_col
                row, col = next_cell
                if not (0 <= row < rows and 0 <= col < cols):
                    continue
                if occupied[row][col]:
                    continue

                new_cost = cost_so_far[current] + step_cost
                if (
                    next_cell not in cost_so_far
                    or new_cost < cost_so_far[next_cell]
                ):
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + self._cell_distance(next_cell, goal)
                    heapq.heappush(open_heap, (priority, next_cell))
                    came_from[next_cell] = current

        return []

    def _cell_distance(
        self,
        first: tuple[int, int],
        second: tuple[int, int],
    ) -> float:
        """Euclidean grid heuristic."""

        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """Reconstruct an A* path."""

        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _smooth_path(
        self,
        points: list[tuple[float, float]],
        occupied: list[list[bool]],
        bounds: tuple[float, float, float, float],
        resolution: float,
    ) -> list[tuple[float, float]]:
        """Remove unnecessary intermediate points with line-of-sight checks."""

        if len(points) <= 2:
            return points

        smoothed = [points[0]]
        anchor_index = 0
        while anchor_index < len(points) - 1:
            next_index = len(points) - 1
            while next_index > anchor_index + 1:
                if self._line_is_free(
                    points[anchor_index],
                    points[next_index],
                    occupied,
                    bounds,
                    resolution,
                ):
                    break
                next_index -= 1
            smoothed.append(points[next_index])
            anchor_index = next_index
        return smoothed

    def _line_is_free(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        occupied: list[list[bool]],
        bounds: tuple[float, float, float, float],
        resolution: float,
    ) -> bool:
        """Return whether a straight segment crosses occupied cells."""

        min_x, _, min_y, _ = bounds
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(int(distance / max(resolution * 0.5, 0.1)), 1)
        rows = len(occupied)
        cols = len(occupied[0]) if rows else 0
        for index in range(steps + 1):
            ratio = index / float(steps)
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            row, col = self._point_to_cell((x, y), min_x, min_y, resolution)
            if not (0 <= row < rows and 0 <= col < cols):
                return False
            if occupied[row][col]:
                return False
        return True

    def _thin_waypoints(
        self,
        points: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        """Keep route points spaced enough for PX4 waypoint following."""

        if len(points) <= 2:
            return points

        spacing = max(
            self.config.waypoint_spacing,
            self.config.grid_resolution,
        )
        thinned = [points[0]]
        for point in points[1:-1]:
            previous = thinned[-1]
            distance = math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            )
            if distance >= spacing:
                thinned.append(point)
        thinned.append(points[-1])
        return thinned
