"""OpenCV top-down map viewer for a Gazebo SDF world."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from xml.etree import ElementTree as ET

import cv2
import numpy as np
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


@dataclass(frozen=True)
class MapObject:
    """Single non-person object projected onto the top-down map."""

    name: str
    uri: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float


class TopDownMapViewer(Node):
    """Draw a bounded top-down OpenCV map from the active Gazebo world."""

    def __init__(self):
        super().__init__('top_down_map_viewer')

        self._declare_parameters()
        self._load_parameters()

        self.objects = self._load_map_objects()
        self.bounds = self._compute_bounds(self.objects)
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.evacuation_route_points: list[tuple[float, float]] = []
        self.current_heading = 0.0
        self.window_available = True

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            'mission/evacuation_route',
            self._evacuation_route_callback,
            10,
        )
        self.timer = self.create_timer(
            max(self.refresh_period_sec, 0.05),
            self._draw_frame,
        )

        self.get_logger().info(
            'Top-down map viewer started: '
            f'world={self.world_sdf_path}, '
            f'objects={len(self.objects)}, '
            f'bounds={self.bounds}'
        )

    def _declare_parameters(self):
        """Declare ROS 2 parameters."""

        self.declare_parameter(
            'world_sdf_path',
            '/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf',
        )
        self.declare_parameter(
            'model_search_paths',
            [
                '/home/dron/.gz/models',
                '/home/dron/.gz/models/model_ruble',
                '/home/dron/.gz/models/model_people',
                '/home/dron/PX4-Autopilot/Tools/simulation/gz/models',
            ],
        )
        self.declare_parameter('window_name', 'Gazebo Top-Down Map')
        self.declare_parameter('window_width', 1100)
        self.declare_parameter('window_height', 850)
        self.declare_parameter('map_padding_m', 5.0)
        self.declare_parameter('object_fallback_size_m', 2.5)
        self.declare_parameter('min_map_size_m', 20.0)
        self.declare_parameter('refresh_period_sec', 0.1)
        self.declare_parameter('draw_object_labels', True)
        self.declare_parameter('draw_axes', True)
        self.declare_parameter('draw_evacuation_route', True)
        self.declare_parameter(
            'person_keywords',
            ['man', 'men', 'person', 'people', 'victim', 'walker'],
        )
        self.declare_parameter('excluded_model_names', ['grave_1'])
        self.declare_parameter('world_swap_xy', False)
        self.declare_parameter('world_invert_x', False)
        self.declare_parameter('world_invert_y', False)
        self.declare_parameter('world_rotation_deg', 0.0)
        self.declare_parameter('world_offset_x', 0.0)
        self.declare_parameter('world_offset_y', 0.0)
        self.declare_parameter('mission_swap_xy', False)
        self.declare_parameter('mission_invert_x', False)
        self.declare_parameter('mission_invert_y', True)
        self.declare_parameter('mission_rotation_deg', 90.0)
        self.declare_parameter('mission_offset_x', 0.0)
        self.declare_parameter('mission_offset_y', 0.0)

    def _load_parameters(self):
        """Load ROS 2 parameters into fields."""

        self.world_sdf_path = str(
            self.get_parameter('world_sdf_path').value
        ).strip()
        self.model_search_paths = [
            Path(str(path)).expanduser()
            for path in self.get_parameter('model_search_paths').value
            if str(path).strip()
        ]
        self.window_name = str(
            self.get_parameter('window_name').value
        ).strip()
        self.window_width = max(
            int(self.get_parameter('window_width').value),
            320,
        )
        self.window_height = max(
            int(self.get_parameter('window_height').value),
            240,
        )
        self.map_padding_m = max(
            float(self.get_parameter('map_padding_m').value),
            0.0,
        )
        self.object_fallback_size_m = max(
            float(self.get_parameter('object_fallback_size_m').value),
            0.1,
        )
        self.min_map_size_m = max(
            float(self.get_parameter('min_map_size_m').value),
            1.0,
        )
        self.refresh_period_sec = float(
            self.get_parameter('refresh_period_sec').value
        )
        self.draw_object_labels = bool(
            self.get_parameter('draw_object_labels').value
        )
        self.draw_axes = bool(self.get_parameter('draw_axes').value)
        self.draw_evacuation_route = bool(
            self.get_parameter('draw_evacuation_route').value
        )
        self.person_keywords = [
            str(keyword).strip().lower()
            for keyword in self.get_parameter('person_keywords').value
            if str(keyword).strip()
        ]
        self.excluded_model_names = {
            str(name).strip().lower()
            for name in self.get_parameter('excluded_model_names').value
            if str(name).strip()
        }
        self.world_swap_xy = bool(self.get_parameter('world_swap_xy').value)
        self.world_invert_x = bool(self.get_parameter('world_invert_x').value)
        self.world_invert_y = bool(self.get_parameter('world_invert_y').value)
        self.world_rotation_deg = float(
            self.get_parameter('world_rotation_deg').value
        )
        self.world_offset_x = float(
            self.get_parameter('world_offset_x').value
        )
        self.world_offset_y = float(
            self.get_parameter('world_offset_y').value
        )
        self.mission_swap_xy = bool(
            self.get_parameter('mission_swap_xy').value
        )
        self.mission_invert_x = bool(
            self.get_parameter('mission_invert_x').value
        )
        self.mission_invert_y = bool(
            self.get_parameter('mission_invert_y').value
        )
        self.mission_rotation_deg = float(
            self.get_parameter('mission_rotation_deg').value
        )
        self.mission_offset_x = float(
            self.get_parameter('mission_offset_x').value
        )
        self.mission_offset_y = float(
            self.get_parameter('mission_offset_y').value
        )

    def _load_map_objects(self) -> list[MapObject]:
        """Read the world file and collect all non-person includes."""

        world_path = Path(self.world_sdf_path).expanduser()
        if not world_path.is_file():
            self.get_logger().error(f'World SDF not found: {world_path}')
            return []

        try:
            root = ET.fromstring(self._read_xml(world_path))
        except ET.ParseError as error:
            self.get_logger().error(
                f'Failed to parse world SDF {world_path}: {error}'
            )
            return []

        world_node = root.find('world')
        if world_node is None:
            self.get_logger().warning(
                f'No <world> element found in {world_path}'
            )
            return []

        objects: list[MapObject] = []
        for include_node in world_node.findall('include'):
            uri = self._child_text(include_node, 'uri')
            name = self._child_text(include_node, 'name')
            name = name or self._name_from_uri(uri)
            if name.lower() in self.excluded_model_names:
                continue
            if self._is_person_model(name, uri):
                continue

            pose = self._parse_pose(self._child_text(include_node, 'pose'))
            size_x, size_y = self._resolve_model_size(uri)
            map_x, map_y = self._world_to_map_xy(pose[0], pose[1])
            map_yaw = self._world_yaw_to_map_yaw(
                pose[0],
                pose[1],
                pose[5],
            )
            objects.append(
                MapObject(
                    name=name,
                    uri=uri,
                    x=map_x,
                    y=map_y,
                    yaw=map_yaw,
                    size_x=size_x,
                    size_y=size_y,
                )
            )

        return objects

    def _read_xml(self, path: Path) -> str:
        """Read XML and repair a known malformed comment style."""

        text = path.read_text(encoding='utf-8', errors='replace')
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

    def _name_from_uri(self, uri: str) -> str:
        """Extract a readable model name from model:// URI."""

        if not uri:
            return 'object'
        return uri.split('://', maxsplit=1)[-1].strip('/').split('/')[-1]

    def _is_person_model(self, name: str, uri: str) -> bool:
        """Return whether a model should be excluded as a person."""

        source = f'{name} {uri}'.lower().replace('_', ' ')
        tokens = [token for token in re.split(r'[^a-z0-9]+', source) if token]
        return any(keyword in tokens for keyword in self.person_keywords)

    def _parse_pose(self, pose_text: str) -> tuple[float, ...]:
        """Parse x y z roll pitch yaw from an SDF pose string."""

        values = [0.0] * 6
        for index, value in enumerate(pose_text.split()[:6]):
            try:
                values[index] = float(value)
            except ValueError:
                values[index] = 0.0
        return tuple(values)

    def _world_to_map_xy(self, x: float, y: float) -> tuple[float, float]:
        """Map Gazebo world XY into the viewer map frame."""

        if self.world_swap_xy:
            x, y = y, x
        if self.world_invert_x:
            x = -x
        if self.world_invert_y:
            y = -y

        rotation_rad = math.radians(self.world_rotation_deg)
        rotated_x = (
            x * math.cos(rotation_rad) - y * math.sin(rotation_rad)
        )
        rotated_y = (
            x * math.sin(rotation_rad) + y * math.cos(rotation_rad)
        )
        return (
            rotated_x + self.world_offset_x,
            rotated_y + self.world_offset_y,
        )

    def _mission_to_map_xy(self, x: float, y: float) -> tuple[float, float]:
        """Map PX4 mission-local XY into the viewer map frame."""

        if self.mission_swap_xy:
            x, y = y, x
        if self.mission_invert_x:
            x = -x
        if self.mission_invert_y:
            y = -y

        rotation_rad = math.radians(self.mission_rotation_deg)
        rotated_x = (
            x * math.cos(rotation_rad) - y * math.sin(rotation_rad)
        )
        rotated_y = (
            x * math.sin(rotation_rad) + y * math.cos(rotation_rad)
        )
        return (
            rotated_x + self.mission_offset_x,
            rotated_y + self.mission_offset_y,
        )

    def _world_yaw_to_map_yaw(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> float:
        """Transform a Gazebo yaw angle into the viewer map frame."""

        x1, y1 = self._world_to_map_xy(x, y)
        x2, y2 = self._world_to_map_xy(
            x + math.cos(yaw),
            y + math.sin(yaw),
        )
        return math.atan2(y2 - y1, x2 - x1)

    def _mission_heading_to_map_heading(self, heading: float) -> float:
        """Transform PX4 local heading into the viewer map frame."""

        x1, y1 = self._mission_to_map_xy(0.0, 0.0)
        x2, y2 = self._mission_to_map_xy(
            math.cos(heading),
            math.sin(heading),
        )
        return math.atan2(y2 - y1, x2 - x1)

    def _resolve_model_size(self, uri: str) -> tuple[float, float]:
        """Resolve top-down model size from model collision or visual data."""

        model_name = self._name_from_uri(uri)
        model_sdf = self._resolve_model_sdf(uri)
        fallback = self.object_fallback_size_m
        if model_sdf is None:
            return fallback, fallback

        try:
            root = ET.fromstring(self._read_xml(model_sdf))
        except ET.ParseError:
            return fallback, fallback

        sizes: list[tuple[float, float]] = []
        for geometry in root.findall('.//collision/geometry'):
            size = self._geometry_size_xy(geometry)
            if size is not None:
                sizes.append(size)

        if model_name == 'house':
            for geometry in root.findall('.//visual/geometry'):
                size = self._mesh_geometry_size_xy(geometry, model_sdf)
                if size is not None:
                    sizes.append(size)

        if not sizes:
            for geometry in root.findall('.//visual/geometry'):
                size = self._geometry_size_xy(geometry)
                if size is not None:
                    sizes.append(size)
        if not sizes:
            return fallback, fallback

        size_x, size_y = max(sizes, key=lambda item: item[0] * item[1])
        return self._adjust_model_size_for_map(model_name, size_x, size_y)

    def _adjust_model_size_for_map(
        self,
        model_name: str,
        size_x: float,
        size_y: float,
    ) -> tuple[float, float]:
        """Apply map-only footprint corrections for selected models."""

        if model_name == 'destroy_building':
            return size_x / 2.0, size_y / 2.0

        if model_name == 'roof_rubble':
            square_size = max(size_x, size_y)
            return square_size, square_size

        if model_name == 'rubble':
            if size_x >= size_y:
                return size_x / 2.0, size_y
            return size_x, size_y / 2.0

        if model_name == 'mango_tree':
            return size_x / 2.0, size_y / 2.0

        return size_x, size_y

    def _resolve_model_sdf(self, uri: str) -> Path | None:
        """Find model.sdf for a model:// include."""

        model_name = self._name_from_uri(uri)
        candidates = [Path(uri).expanduser()] if uri.startswith('/') else []
        for root in self.model_search_paths:
            candidates.append(root / model_name)
            candidates.extend(root.glob(f'*/{model_name}'))

        for candidate in candidates:
            if candidate.is_file() and candidate.suffix == '.sdf':
                return candidate
            model_sdf = candidate / 'model.sdf'
            if model_sdf.is_file():
                return model_sdf
        return None

    def _geometry_size_xy(
        self,
        geometry_node: ET.Element,
    ) -> tuple[float, float] | None:
        """Return XY footprint for common SDF geometry types."""

        box_size = geometry_node.find('box/size')
        if box_size is not None and box_size.text:
            values = self._parse_float_list(box_size.text)
            if len(values) >= 2:
                return abs(values[0]), abs(values[1])

        radius_node = geometry_node.find('cylinder/radius')
        if radius_node is not None and radius_node.text:
            radius = self._parse_float(radius_node.text)
            if radius is not None:
                diameter = abs(radius) * 2.0
                return diameter, diameter

        sphere_node = geometry_node.find('sphere/radius')
        if sphere_node is not None and sphere_node.text:
            radius = self._parse_float(sphere_node.text)
            if radius is not None:
                diameter = abs(radius) * 2.0
                return diameter, diameter

        return None

    def _mesh_geometry_size_xy(
        self,
        geometry_node: ET.Element,
        model_sdf: Path,
    ) -> tuple[float, float] | None:
        """Return XY footprint for supported mesh visuals."""

        mesh_node = geometry_node.find('mesh')
        if mesh_node is None:
            return None

        mesh_uri = self._child_text(mesh_node, 'uri')
        mesh_path = self._resolve_mesh_path(mesh_uri, model_sdf)
        if mesh_path is None:
            return None

        scale_values = self._parse_float_list(
            self._child_text(mesh_node, 'scale')
        )
        if not scale_values:
            scale_values = [1.0, 1.0, 1.0]
        while len(scale_values) < 3:
            scale_values.append(1.0)

        if mesh_path.suffix.lower() == '.gltf':
            return self._gltf_mesh_size_xy(mesh_path, scale_values)
        return None

    def _resolve_mesh_path(
        self,
        mesh_uri: str,
        model_sdf: Path,
    ) -> Path | None:
        """Resolve SDF mesh URI into a local file path."""

        if not mesh_uri:
            return None
        if mesh_uri.startswith('model://'):
            tail = mesh_uri.split('://', maxsplit=1)[1].strip('/')
            parts = tail.split('/', maxsplit=1)
            if parts and parts[0] == model_sdf.parent.name and len(parts) > 1:
                candidate = model_sdf.parent / parts[1]
                if candidate.is_file():
                    return candidate
            for root in self.model_search_paths:
                candidate = root / tail
                if candidate.is_file():
                    return candidate
                if len(parts) > 1:
                    nested = root / parts[0] / parts[1]
                    if nested.is_file():
                        return nested
                    for matched_root in root.glob(f'*/{parts[0]}'):
                        nested = matched_root / parts[1]
                        if nested.is_file():
                            return nested
            return None

        candidate = Path(mesh_uri).expanduser()
        if candidate.is_file():
            return candidate
        candidate = model_sdf.parent / mesh_uri
        if candidate.is_file():
            return candidate
        return None

    def _gltf_mesh_size_xy(
        self,
        mesh_path: Path,
        sdf_scale: list[float],
    ) -> tuple[float, float] | None:
        """Estimate GLTF scene XY bounds from accessor min/max metadata."""

        try:
            data = json.loads(mesh_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return None

        nodes = data.get('nodes', [])
        meshes = data.get('meshes', [])
        accessors = data.get('accessors', [])
        scenes = data.get('scenes', [])
        if not nodes or not meshes or not accessors or not scenes:
            return None

        scene_index = int(data.get('scene', 0))
        if not 0 <= scene_index < len(scenes):
            return None

        min_bounds = np.array([math.inf, math.inf, math.inf], dtype=float)
        max_bounds = np.array([-math.inf, -math.inf, -math.inf], dtype=float)
        found_bounds = False

        for node_index in scenes[scene_index].get('nodes', []):
            for node, matrix in self._walk_gltf_nodes(
                nodes,
                int(node_index),
                np.eye(4),
            ):
                mesh_index = node.get('mesh')
                if mesh_index is None:
                    continue
                if not 0 <= int(mesh_index) < len(meshes):
                    continue
                mesh = meshes[int(mesh_index)]
                for primitive in mesh.get('primitives', []):
                    attributes = primitive.get('attributes', {})
                    accessor_index = attributes.get('POSITION')
                    if accessor_index is None:
                        continue
                    if not 0 <= int(accessor_index) < len(accessors):
                        continue
                    accessor = accessors[int(accessor_index)]
                    raw_min = accessor.get('min')
                    raw_max = accessor.get('max')
                    if raw_min is None or raw_max is None:
                        continue
                    transformed = self._transform_gltf_bounds(
                        raw_min,
                        raw_max,
                        matrix,
                    )
                    min_bounds = np.minimum(min_bounds, transformed[0])
                    max_bounds = np.maximum(max_bounds, transformed[1])
                    found_bounds = True

        if not found_bounds:
            return None

        dimensions = (max_bounds - min_bounds) * np.array(
            sdf_scale[:3],
            dtype=float,
        )
        return abs(float(dimensions[0])), abs(float(dimensions[1]))

    def _walk_gltf_nodes(
        self,
        nodes: list[dict],
        node_index: int,
        parent_matrix: np.ndarray,
    ):
        """Yield GLTF nodes with accumulated transforms."""

        if not 0 <= node_index < len(nodes):
            return

        node = nodes[node_index]
        matrix = parent_matrix @ self._gltf_node_matrix(node)
        yield node, matrix
        for child_index in node.get('children', []):
            yield from self._walk_gltf_nodes(
                nodes,
                int(child_index),
                matrix,
            )

    def _gltf_node_matrix(self, node: dict) -> np.ndarray:
        """Return a node transform matrix for common GLTF transform fields."""

        if 'matrix' in node:
            return np.array(node['matrix'], dtype=float).reshape(
                (4, 4),
                order='F',
            )

        matrix = np.eye(4)
        if 'translation' in node:
            matrix[:3, 3] = np.array(node['translation'][:3], dtype=float)
        if 'scale' in node:
            scale = list(node['scale'][:3])
            while len(scale) < 3:
                scale.append(1.0)
            matrix = matrix @ np.diag([scale[0], scale[1], scale[2], 1.0])
        return matrix

    def _transform_gltf_bounds(
        self,
        raw_min: list[float],
        raw_max: list[float],
        matrix: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Transform accessor min/max corners and return new bounds."""

        points = []
        for x in (float(raw_min[0]), float(raw_max[0])):
            for y in (float(raw_min[1]), float(raw_max[1])):
                for z in (float(raw_min[2]), float(raw_max[2])):
                    point = matrix @ np.array([x, y, z, 1.0], dtype=float)
                    points.append(point[:3])

        transformed = np.array(points)
        return transformed.min(axis=0), transformed.max(axis=0)

    def _parse_float_list(self, text: str) -> list[float]:
        """Parse a whitespace separated float list."""

        values = []
        for raw_value in text.split():
            parsed = self._parse_float(raw_value)
            if parsed is not None:
                values.append(parsed)
        return values

    def _parse_float(self, text: str) -> float | None:
        """Parse one float value."""

        try:
            return float(text)
        except ValueError:
            return None

    def _compute_bounds(
        self,
        objects: list[MapObject],
    ) -> tuple[float, float, float, float]:
        """Compute finite map bounds from object footprints."""

        if not objects:
            half = self.min_map_size_m / 2.0
            return -half, half, -half, half

        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        for obj in objects:
            half_x = max(obj.size_x / 2.0, 0.1)
            half_y = max(obj.size_y / 2.0, 0.1)
            min_x = min(min_x, obj.x - half_x)
            max_x = max(max_x, obj.x + half_x)
            min_y = min(min_y, obj.y - half_y)
            max_y = max(max_y, obj.y + half_y)

        min_x -= self.map_padding_m
        max_x += self.map_padding_m
        min_y -= self.map_padding_m
        max_y += self.map_padding_m

        width = max_x - min_x
        height = max_y - min_y
        if width < self.min_map_size_m:
            extra = (self.min_map_size_m - width) / 2.0
            min_x -= extra
            max_x += extra
        if height < self.min_map_size_m:
            extra = (self.min_map_size_m - height) / 2.0
            min_y -= extra
            max_y += extra
        return min_x, max_x, min_y, max_y

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Store latest vehicle position for live overlay."""

        self.vehicle_local_position = msg
        if math.isfinite(msg.heading):
            self.current_heading = float(msg.heading)

    def _evacuation_route_callback(self, msg: Float32MultiArray):
        """Store latest A* evacuation route in map coordinates."""

        data = list(msg.data)
        if len(data) < 4:
            self.evacuation_route_points = []
            return

        route_points = []
        for index in range(0, len(data) - 1, 2):
            route_points.append(
                self._mission_to_map_xy(
                    float(data[index]),
                    float(data[index + 1]),
                )
            )
        self.evacuation_route_points = route_points

    def _draw_frame(self):
        """Render one map frame."""

        if not self.window_available:
            return

        frame = np.full(
            (self.window_height, self.window_width, 3),
            (245, 245, 240),
            dtype=np.uint8,
        )
        self._draw_grid(frame)
        self._draw_objects(frame)
        self._draw_evacuation_route(frame)
        self._draw_vehicle(frame)
        self._draw_header(frame)

        try:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except cv2.error as error:
            self.window_available = False
            self.get_logger().error(
                'Could not open OpenCV map window. '
                f'Check DISPLAY or run with GUI access. Reason: {error}'
            )

    def _world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        """Convert world XY into image coordinates."""

        min_x, max_x, min_y, max_y = self.bounds
        width_m = max(max_x - min_x, 1e-6)
        height_m = max(max_y - min_y, 1e-6)
        scale = min(
            (self.window_width - 80) / width_m,
            (self.window_height - 80) / height_m,
        )
        map_width_px = width_m * scale
        map_height_px = height_m * scale
        offset_x = (self.window_width - map_width_px) / 2.0
        offset_y = (self.window_height - map_height_px) / 2.0

        px = offset_x + (x - min_x) * scale
        py = offset_y + (max_y - y) * scale
        return int(round(px)), int(round(py))

    def _meters_to_pixels(self, meters: float) -> int:
        """Convert meters to pixels using the current map scale."""

        x0, _ = self._world_to_pixel(0.0, 0.0)
        x1, _ = self._world_to_pixel(meters, 0.0)
        return max(abs(x1 - x0), 1)

    def _draw_grid(self, frame: np.ndarray):
        """Draw map boundary, grid and axes."""

        min_x, max_x, min_y, max_y = self.bounds
        p_min = self._world_to_pixel(min_x, min_y)
        p_max = self._world_to_pixel(max_x, max_y)
        x1, x2 = sorted((p_min[0], p_max[0]))
        y1, y2 = sorted((p_min[1], p_max[1]))
        cv2.rectangle(frame, (x1, y1), (x2, y2), (80, 80, 80), 2)

        step = 5.0
        start_x = math.floor(min_x / step) * step
        value = start_x
        while value <= max_x:
            px, _ = self._world_to_pixel(value, min_y)
            cv2.line(frame, (px, y1), (px, y2), (220, 220, 215), 1)
            value += step

        start_y = math.floor(min_y / step) * step
        value = start_y
        while value <= max_y:
            _, py = self._world_to_pixel(min_x, value)
            cv2.line(frame, (x1, py), (x2, py), (220, 220, 215), 1)
            value += step

        if self.draw_axes:
            ox, oy = self._world_to_pixel(0.0, 0.0)
            cv2.line(frame, (x1, oy), (x2, oy), (170, 170, 170), 1)
            cv2.line(frame, (ox, y1), (ox, y2), (170, 170, 170), 1)
            cv2.putText(
                frame,
                'X',
                (x2 - 20, oy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (80, 80, 80),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                'Y',
                (ox + 8, y1 + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (80, 80, 80),
                1,
                cv2.LINE_AA,
            )

    def _draw_objects(self, frame: np.ndarray):
        """Draw all static non-person objects."""

        for obj in self.objects:
            center = self._world_to_pixel(obj.x, obj.y)
            half_x = self._meters_to_pixels(obj.size_x) / 2.0
            half_y = self._meters_to_pixels(obj.size_y) / 2.0
            corners = self._rotated_box_corners(
                center,
                half_x,
                half_y,
                -obj.yaw,
            )
            cv2.fillConvexPoly(frame, corners, (155, 135, 105))
            cv2.polylines(frame, [corners], True, (80, 65, 45), 2)

            if self.draw_object_labels:
                cv2.putText(
                    frame,
                    obj.name[:28],
                    (center[0] + 5, center[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (40, 40, 40),
                    1,
                    cv2.LINE_AA,
                )

    def _draw_evacuation_route(self, frame: np.ndarray):
        """Draw the latest A* evacuation route."""

        if not self.draw_evacuation_route:
            return
        if len(self.evacuation_route_points) < 2:
            return

        pixel_points = [
            self._world_to_pixel(x, y)
            for x, y in self.evacuation_route_points
        ]
        polyline = np.array(pixel_points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(
            frame,
            [polyline],
            False,
            (0, 190, 255),
            4,
            cv2.LINE_AA,
        )

        for index, point in enumerate(pixel_points):
            radius = 7 if index in {0, len(pixel_points) - 1} else 5
            cv2.circle(frame, point, radius, (0, 120, 255), -1)
            cv2.circle(frame, point, radius, (255, 255, 255), 2)

        start = pixel_points[0]
        cv2.putText(
            frame,
            'A* route',
            (start[0] + 8, start[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 90, 180),
            2,
            cv2.LINE_AA,
        )

    def _rotated_box_corners(
        self,
        center: tuple[int, int],
        half_x: float,
        half_y: float,
        yaw: float,
    ) -> np.ndarray:
        """Return rotated rectangle corners in pixel coordinates."""

        points = [(-half_x, -half_y), (half_x, -half_y)]
        points.extend([(half_x, half_y), (-half_x, half_y)])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        corners = []
        for x, y in points:
            px = center[0] + x * cos_yaw - y * sin_yaw
            py = center[1] + x * sin_yaw + y * cos_yaw
            corners.append([int(round(px)), int(round(py))])
        return np.array(corners, dtype=np.int32)

    def _draw_vehicle(self, frame: np.ndarray):
        """Draw current drone position and heading."""

        if self.vehicle_local_position is None:
            return
        if not self.vehicle_local_position.xy_valid:
            return

        x, y = self._mission_to_map_xy(
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
        )
        center = self._world_to_pixel(x, y)
        radius = max(self._meters_to_pixels(0.8), 8)
        heading = self._mission_heading_to_map_heading(self.current_heading)
        tip = (
            int(center[0] + math.cos(-heading) * radius * 1.7),
            int(center[1] + math.sin(-heading) * radius * 1.7),
        )

        cv2.circle(frame, center, radius, (30, 110, 230), -1)
        cv2.circle(frame, center, radius, (15, 55, 120), 2)
        cv2.line(frame, center, tip, (15, 55, 120), 3)
        cv2.putText(
            frame,
            'drone',
            (center[0] + radius + 4, center[1] + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (15, 55, 120),
            2,
            cv2.LINE_AA,
        )

    def _draw_header(self, frame: np.ndarray):
        """Draw compact map status text."""

        min_x, max_x, min_y, max_y = self.bounds
        text = (
            f'objects: {len(self.objects)}  '
            f'x: {min_x:.1f}..{max_x:.1f} m  '
            f'y: {min_y:.1f}..{max_y:.1f} m'
        )
        cv2.putText(
            frame,
            text,
            (18, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (35, 35, 35),
            2,
            cv2.LINE_AA,
        )


def main(args=None):
    """Run the top-down map viewer node."""

    rclpy.init(args=args)
    node = TopDownMapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping top-down map viewer')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
