"""Publish a top-down RViz scene map from Gazebo SDF and PX4 state."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import subprocess
from xml.etree import ElementTree as ET

from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from offboard_takeoff.mission import default_mission_plan


@dataclass(frozen=True)
class SceneObject:
    """Static scene object projected into the mission map frame."""

    name: str
    source_uri: str
    x: float
    y: float
    z: float
    yaw: float
    size_x: float
    size_y: float
    size_z: float


class SceneMapPublisher(Node):
    """Visualize the Gazebo world, mission path, and drone pose in RViz."""

    def __init__(self):
        super().__init__('scene_map_publisher')

        self._declare_parameters()
        self._load_parameters()

        self.mission_plan = default_mission_plan(self.takeoff_height)
        self.scene_objects, self.person_objects = self._load_scene_objects()
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.current_heading = 0.0
        self.trail_points: list[Point] = []
        self.detection_points: list[Point] = []
        self.last_target_detected = False
        self.rescued_person_names: set[str] = set()
        self.rescued_person_count = 0
        self._publish_static_map_transform()

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'scene_map/markers',
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
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            Bool,
            'yolo/target_detected',
            self._target_detected_callback,
            10,
        )
        self.create_subscription(
            Bool,
            'mission/person_rescued',
            self._person_rescued_callback,
            10,
        )

        self.timer = self.create_timer(
            max(self.publish_period_sec, 0.1),
            self._publish_markers,
        )

        self.get_logger().info(
            'Scene map publisher started: '
            f'frame_id={self.frame_id}, '
            f'world_sdf_path={self.world_sdf_path}, '
            f'scene_objects={len(self.scene_objects)}, '
            f'person_objects={len(self.person_objects)}'
        )

    def _declare_parameters(self):
        """Declare ROS 2 parameters for map rendering and SDF parsing."""

        self.declare_parameter(
            'world_sdf_path',
            '/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf',
        )
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('frame_id', 'mission_map')
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('takeoff_height', 5.0)
        self.declare_parameter('path_line_width', 0.2)
        self.declare_parameter('trail_line_width', 0.12)
        self.declare_parameter('object_alpha', 0.85)
        self.declare_parameter('object_size_xy', 3.5)
        self.declare_parameter('object_size_z', 2.0)
        self.declare_parameter('house_size_x', 16.27)
        self.declare_parameter('house_size_y', 6.07)
        self.declare_parameter('house_size_z', 9.90)
        self.declare_parameter('building_size_x', 30.58)
        self.declare_parameter('building_size_y', 28.97)
        self.declare_parameter('building_size_z', 8.18)
        self.declare_parameter('roof_rubble_size_xy', 6.03)
        self.declare_parameter('roof_rubble_size_z', 6.72)
        self.declare_parameter('rubble_size_x', 10.79)
        self.declare_parameter('rubble_size_y', 4.09)
        self.declare_parameter('rubble_size_z', 12.22)
        self.declare_parameter('drone_scale', 1.2)
        self.declare_parameter('trail_max_points', 500)
        self.declare_parameter('record_detection_points', True)
        self.declare_parameter(
            'debris_keywords',
            [
                'rubble',
                'destroy',
                'debris',
                'collapsed',
                'ruin',
                'wreck',
                'house',
                'building',
                'mango_tree',
            ],
        )
        self.declare_parameter(
            'person_keywords',
            [
                'man',
                'men',
                'person',
                'people',
                'victim',
                'walker',
            ],
        )
        self.declare_parameter('teleport_rescued_people', True)
        self.declare_parameter('hide_rescued_people', True)
        self.declare_parameter('rescue_service_timeout_ms', 2000)
        self.declare_parameter('teleport_world_name', '')
        self.declare_parameter('rescued_person_teleport_x', 200.0)
        self.declare_parameter('rescued_person_teleport_y', 200.0)
        self.declare_parameter('rescued_person_teleport_z', 0.5)
        self.declare_parameter('rescued_person_teleport_spacing', 3.0)
        self.declare_parameter('world_swap_xy', False)
        self.declare_parameter('world_invert_x', False)
        self.declare_parameter('world_invert_y', False)
        self.declare_parameter('world_rotation_deg', 0.0)
        self.declare_parameter('world_offset_x', 0.0)
        self.declare_parameter('world_offset_y', 0.0)
        self.declare_parameter('mission_rotation_deg', 90.0)
        self.declare_parameter('mission_offset_x', 0.0)
        self.declare_parameter('mission_offset_y', 0.0)

    def _load_parameters(self):
        """Load parameters into node fields."""

        self.world_sdf_path = str(
            self.get_parameter('world_sdf_path').value
        ).strip()
        self.parent_frame_id = str(
            self.get_parameter('parent_frame_id').value
        ).strip()
        self.frame_id = str(self.get_parameter('frame_id').value).strip()
        self.publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self.takeoff_height = float(
            self.get_parameter('takeoff_height').value
        )
        self.path_line_width = float(
            self.get_parameter('path_line_width').value
        )
        self.trail_line_width = float(
            self.get_parameter('trail_line_width').value
        )
        self.object_alpha = float(
            self.get_parameter('object_alpha').value
        )
        self.object_size_xy = float(
            self.get_parameter('object_size_xy').value
        )
        self.object_size_z = float(
            self.get_parameter('object_size_z').value
        )
        self.house_size_x = float(
            self.get_parameter('house_size_x').value
        )
        self.house_size_y = float(
            self.get_parameter('house_size_y').value
        )
        self.house_size_z = float(
            self.get_parameter('house_size_z').value
        )
        self.building_size_x = float(
            self.get_parameter('building_size_x').value
        )
        self.building_size_y = float(
            self.get_parameter('building_size_y').value
        )
        self.building_size_z = float(
            self.get_parameter('building_size_z').value
        )
        self.roof_rubble_size_xy = float(
            self.get_parameter('roof_rubble_size_xy').value
        )
        self.roof_rubble_size_z = float(
            self.get_parameter('roof_rubble_size_z').value
        )
        self.rubble_size_x = float(
            self.get_parameter('rubble_size_x').value
        )
        self.rubble_size_y = float(
            self.get_parameter('rubble_size_y').value
        )
        self.rubble_size_z = float(
            self.get_parameter('rubble_size_z').value
        )
        self.drone_scale = float(self.get_parameter('drone_scale').value)
        self.trail_max_points = max(
            int(self.get_parameter('trail_max_points').value),
            1,
        )
        self.record_detection_points = bool(
            self.get_parameter('record_detection_points').value
        )
        self.debris_keywords = [
            str(keyword).strip().lower()
            for keyword in self.get_parameter('debris_keywords').value
            if str(keyword).strip()
        ]
        self.person_keywords = [
            str(keyword).strip().lower()
            for keyword in self.get_parameter('person_keywords').value
            if str(keyword).strip()
        ]
        self.teleport_rescued_people = bool(
            self.get_parameter('teleport_rescued_people').value
        )
        self.hide_rescued_people = bool(
            self.get_parameter('hide_rescued_people').value
        )
        self.rescue_service_timeout_ms = max(
            int(self.get_parameter('rescue_service_timeout_ms').value),
            100,
        )
        self.teleport_world_name = str(
            self.get_parameter('teleport_world_name').value
        ).strip()
        self.rescued_person_teleport_x = float(
            self.get_parameter('rescued_person_teleport_x').value
        )
        self.rescued_person_teleport_y = float(
            self.get_parameter('rescued_person_teleport_y').value
        )
        self.rescued_person_teleport_z = float(
            self.get_parameter('rescued_person_teleport_z').value
        )
        self.rescued_person_teleport_spacing = float(
            self.get_parameter('rescued_person_teleport_spacing').value
        )
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
        self.mission_rotation_deg = float(
            self.get_parameter('mission_rotation_deg').value
        )
        self.mission_offset_x = float(
            self.get_parameter('mission_offset_x').value
        )
        self.mission_offset_y = float(
            self.get_parameter('mission_offset_y').value
        )

    def _publish_static_map_transform(self):
        """Publish an identity transform so RViz has a valid fixed frame."""

        if not self.parent_frame_id or not self.frame_id:
            return
        if self.parent_frame_id == self.frame_id:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame_id
        transform.child_frame_id = self.frame_id
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _load_scene_objects(
        self,
    ) -> tuple[list[SceneObject], list[SceneObject]]:
        """Parse the configured world file and collect debris and people."""

        world_path = Path(self.world_sdf_path).expanduser()
        if not world_path.is_file():
            self.get_logger().warning(
                f'World SDF file was not found: {world_path}'
            )
            return [], []

        try:
            root = ET.fromstring(self._read_world_sdf_xml(world_path))
        except ET.ParseError as error:
            self.get_logger().error(
                f'Failed to parse world SDF file {world_path}: {error}'
            )
            return [], []

        objects: list[SceneObject] = []
        people: list[SceneObject] = []
        world_node = root.find('world')
        if world_node is None:
            self.get_logger().warning(
                f'No <world> element was found in {world_path}'
            )
            return [], []

        for include_node in world_node.findall('include'):
            uri = self._child_text(include_node, 'uri')
            explicit_name = self._child_text(include_node, 'name')
            resolved_name = explicit_name or self._name_from_uri(uri)
            source_text = f'{resolved_name} {uri}'.lower()
            is_debris = self._matches_keywords(
                source_text,
                self.debris_keywords,
            )
            is_person = self._matches_keywords(
                source_text,
                self.person_keywords,
            )
            if not is_debris and not is_person:
                continue

            pose_values = self._parse_pose_text(
                self._child_text(include_node, 'pose')
            )
            x, y = self._world_to_map_xy(pose_values[0], pose_values[1])
            size_x, size_y, size_z = self._scene_object_size(
                resolved_name,
                uri,
            )
            scene_object = SceneObject(
                name=explicit_name or resolved_name,
                source_uri=uri,
                x=x,
                y=y,
                z=pose_values[2],
                yaw=pose_values[5],
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
            )
            if is_person:
                people.append(scene_object)
            elif is_debris:
                objects.append(scene_object)

        self.get_logger().info(
            f'Loaded {len(objects)} debris objects and '
            f'{len(people)} people from {world_path}'
        )
        return objects, people

    def _scene_object_size(
        self,
        name: str,
        uri: str,
    ) -> tuple[float, float, float]:
        """Return marker size for scene objects with local overrides."""

        model_name = self._name_from_uri(uri).lower()
        normalized_name = name.lower()
        if model_name == 'house' or normalized_name.startswith('house'):
            return self.house_size_x, self.house_size_y, self.house_size_z
        if model_name == 'destroy_building':
            return (
                self.building_size_x,
                self.building_size_y,
                self.building_size_z,
            )
        if model_name == 'roof_rubble':
            return (
                self.roof_rubble_size_xy,
                self.roof_rubble_size_xy,
                self.roof_rubble_size_z,
            )
        if model_name == 'rubble':
            return self.rubble_size_x, self.rubble_size_y, self.rubble_size_z

        return self.object_size_xy, self.object_size_xy, self.object_size_z

    def _read_world_sdf_xml(self, world_path: Path) -> str:
        """Read world XML and repair a known malformed block comment."""

        xml_text = world_path.read_text(encoding='utf-8', errors='replace')
        if '<!---' in xml_text and '--->' in xml_text:
            xml_text = xml_text.replace('<!---', '<!--')
            xml_text = xml_text.replace('--->', '-->')
        return xml_text

    def _child_text(self, node: ET.Element, child_name: str) -> str:
        """Return normalized text of a child node or an empty string."""

        child = node.find(child_name)
        if child is None or child.text is None:
            return ''
        return child.text.strip()

    def _name_from_uri(self, uri: str) -> str:
        """Extract a readable model name from an SDF include URI."""

        if not uri:
            return 'object'
        return uri.split('://', maxsplit=1)[-1].strip('/').split('/')[-1]

    def _matches_keywords(
        self,
        text: str,
        keywords: list[str],
    ) -> bool:
        """Return whether a model name or URI matches a keyword."""

        normalized_tokens = [
            token
            for token in re.split(r'[^a-z0-9]+', text.replace('_', ' '))
            if token
        ]
        return any(keyword in normalized_tokens for keyword in keywords)

    def _parse_pose_text(self, pose_text: str) -> tuple[float, ...]:
        """Parse an SDF pose string into x y z roll pitch yaw."""

        default_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if not pose_text:
            return default_pose

        parts = pose_text.split()
        values = [0.0] * 6
        for index, value in enumerate(parts[:6]):
            try:
                values[index] = float(value)
            except ValueError:
                values[index] = 0.0
        return tuple(values)

    def _world_to_map_xy(self, x: float, y: float) -> tuple[float, float]:
        """Map Gazebo world XY into the mission map frame."""

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
        """Map PX4 mission-local XY into the RViz map frame."""

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

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Store the latest PX4 local position and extend the drone trail."""

        self.vehicle_local_position = msg
        if math.isfinite(msg.heading):
            self.current_heading = float(msg.heading)

        point = Point(
            x=0.0,
            y=0.0,
            z=0.05,
        )
        point.x, point.y = self._mission_to_map_xy(
            float(msg.x),
            float(msg.y),
        )
        if self.trail_points:
            previous = self.trail_points[-1]
            if math.hypot(point.x - previous.x, point.y - previous.y) < 0.1:
                return
        self.trail_points.append(point)
        if len(self.trail_points) > self.trail_max_points:
            self.trail_points = self.trail_points[-self.trail_max_points:]

    def _target_detected_callback(self, msg: Bool):
        """Record a detection point at the drone position on rising edges."""

        detected = bool(msg.data)
        if (
            detected
            and not self.last_target_detected
            and self.record_detection_points
            and self.vehicle_local_position is not None
        ):
            self.detection_points.append(
                self._make_point_from_mission_xy(
                    float(self.vehicle_local_position.x),
                    float(self.vehicle_local_position.y),
                    z=0.15,
                )
            )
        self.last_target_detected = detected

    def _person_rescued_callback(self, msg: Bool):
        """Hide and optionally teleport the nearest unrescued person model."""

        if not bool(msg.data):
            return

        rescued_person = self._nearest_active_person()
        if rescued_person is None:
            self.get_logger().warning(
                'Received person_rescued event but no active person model '
                'was available'
            )
            return

        teleported = True
        if self.teleport_rescued_people:
            teleported = self._teleport_person_out_of_scene(rescued_person)
        if self.hide_rescued_people or teleported:
            self.rescued_person_names.add(rescued_person.name)

        status = (
            'teleported out of scene'
            if teleported
            else 'hidden on map only'
        )
        self.get_logger().info(
            f'Rescued person {rescued_person.name}: {status}'
        )

    def _publish_markers(self):
        """Publish the full marker array used by RViz."""

        markers = [
            *self._build_scene_object_markers(),
            *self._build_person_object_markers(),
            self._build_search_path_marker(),
            self._build_waypoint_marker(),
            self._build_search_area_marker(),
            self._build_home_marker(),
            self._build_trail_marker(),
            self._build_detection_marker(),
        ]

        drone_marker = self._build_drone_marker()
        if drone_marker is not None:
            markers.append(drone_marker)

        self.marker_pub.publish(MarkerArray(markers=markers))

    def _build_scene_object_markers(self) -> list[Marker]:
        """Build markers for static debris objects."""

        stamp = self.get_clock().now().to_msg()
        markers: list[Marker] = []
        for index, scene_object in enumerate(self.scene_objects):
            cube_marker = Marker()
            cube_marker.header.frame_id = self.frame_id
            cube_marker.header.stamp = stamp
            cube_marker.ns = 'scene_objects'
            cube_marker.id = index
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose.position.x = scene_object.x
            cube_marker.pose.position.y = scene_object.y
            cube_marker.pose.position.z = max(scene_object.size_z / 2.0, 0.1)
            cube_marker.pose.orientation.z = math.sin(scene_object.yaw / 2.0)
            cube_marker.pose.orientation.w = math.cos(scene_object.yaw / 2.0)
            cube_marker.scale.x = scene_object.size_x
            cube_marker.scale.y = scene_object.size_y
            cube_marker.scale.z = scene_object.size_z
            cube_marker.color.r = 0.55
            cube_marker.color.g = 0.35
            cube_marker.color.b = 0.20
            cube_marker.color.a = self.object_alpha
            markers.append(cube_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = 'scene_object_labels'
            text_marker.id = 1000 + index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = scene_object.x
            text_marker.pose.position.y = scene_object.y
            text_marker.pose.position.z = scene_object.size_z + 0.8
            text_marker.scale.z = 0.8
            text_marker.color.r = 1.0
            text_marker.color.g = 0.95
            text_marker.color.b = 0.85
            text_marker.color.a = 0.95
            text_marker.text = scene_object.name
            markers.append(text_marker)
        return markers

    def _build_person_object_markers(self) -> list[Marker]:
        """Build markers for people that are still active in the scene."""

        stamp = self.get_clock().now().to_msg()
        markers: list[Marker] = []
        visible_people = [
            scene_object
            for scene_object in self.person_objects
            if scene_object.name not in self.rescued_person_names
        ]
        for index, scene_object in enumerate(visible_people):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'scene_people'
            marker.id = 3000 + index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = scene_object.x
            marker.pose.position.y = scene_object.y
            marker.pose.position.z = max(scene_object.size_z / 2.0, 0.1)
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = max(scene_object.size_z, 1.6)
            marker.color.r = 0.95
            marker.color.g = 0.15
            marker.color.b = 0.15
            marker.color.a = 0.95
            markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = 'scene_people_labels'
            text_marker.id = 4000 + index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = scene_object.x
            text_marker.pose.position.y = scene_object.y
            text_marker.pose.position.z = max(scene_object.size_z, 1.6) + 0.8
            text_marker.scale.z = 0.8
            text_marker.color.r = 1.0
            text_marker.color.g = 0.95
            text_marker.color.b = 0.9
            text_marker.color.a = 0.95
            text_marker.text = scene_object.name
            markers.append(text_marker)
        return markers

    def _build_search_path_marker(self) -> Marker:
        """Build a line strip for the configured mission path."""

        marker = self._make_marker('mission_path', 2000, Marker.LINE_STRIP)
        marker.scale.x = max(self.path_line_width, 0.02)
        marker.color.r = 0.05
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points.append(
            self._make_point_from_mission_xy(
                self.mission_plan.takeoff_waypoint.x,
                self.mission_plan.takeoff_waypoint.y,
                z=0.05,
            )
        )
        for waypoint in self.mission_plan.search_waypoints:
            marker.points.append(
                self._make_point_from_mission_xy(
                    waypoint.x,
                    waypoint.y,
                    z=0.05,
                )
            )
        return marker

    def _build_waypoint_marker(self) -> Marker:
        """Build sphere markers for search waypoints."""

        marker = self._make_marker(
            'mission_waypoints',
            2001,
            Marker.SPHERE_LIST,
        )
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 0.55
        marker.color.r = 0.15
        marker.color.g = 0.85
        marker.color.b = 0.95
        marker.color.a = 0.95
        for waypoint in self.mission_plan.search_waypoints:
            marker.points.append(
                self._make_point_from_mission_xy(
                    waypoint.x,
                    waypoint.y,
                    z=0.10,
                )
            )
        return marker

    def _build_search_area_marker(self) -> Marker:
        """Build a bounding rectangle around the mission waypoints."""

        marker = self._make_marker('search_area', 2002, Marker.LINE_STRIP)
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.15
        marker.color.a = 0.95

        map_points = [
            self._mission_to_map_xy(
                self.mission_plan.takeoff_waypoint.x,
                self.mission_plan.takeoff_waypoint.y,
            )
        ]
        for waypoint in self.mission_plan.search_waypoints:
            map_points.append(self._mission_to_map_xy(waypoint.x, waypoint.y))

        xs = [point[0] for point in map_points]
        ys = [point[1] for point in map_points]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        corners = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
        ]
        for x, y in corners:
            marker.points.append(Point(x=x, y=y, z=0.02))
        return marker

    def _build_home_marker(self) -> Marker:
        """Build a marker for the mission home point."""

        marker = self._make_marker('home', 2003, Marker.CYLINDER)
        marker.pose.position.x, marker.pose.position.y = (
            self._mission_to_map_xy(
                self.mission_plan.takeoff_waypoint.x,
                self.mission_plan.takeoff_waypoint.y,
            )
        )
        marker.pose.position.z = 0.1
        marker.scale.x = 1.1
        marker.scale.y = 1.1
        marker.scale.z = 0.2
        marker.color.r = 0.1
        marker.color.g = 0.95
        marker.color.b = 0.2
        marker.color.a = 0.95
        return marker

    def _build_trail_marker(self) -> Marker:
        """Build the historical flight trail marker."""

        marker = self._make_marker('drone_trail', 2004, Marker.LINE_STRIP)
        marker.scale.x = max(self.trail_line_width, 0.02)
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.15
        marker.color.a = 0.95
        marker.points = list(self.trail_points)
        return marker

    def _build_detection_marker(self) -> Marker:
        """Build the marker for recorded detection points."""

        marker = self._make_marker('detections', 2005, Marker.SPHERE_LIST)
        marker.scale.x = 0.75
        marker.scale.y = 0.75
        marker.scale.z = 0.75
        marker.color.r = 0.95
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.95
        marker.points = list(self.detection_points)
        return marker

    def _build_drone_marker(self) -> Marker | None:
        """Build the current drone pose marker."""

        if self.vehicle_local_position is None:
            return None

        marker = self._make_marker('drone', 2006, Marker.ARROW)
        marker.pose.position.x, marker.pose.position.y = (
            self._mission_to_map_xy(
                float(self.vehicle_local_position.x),
                float(self.vehicle_local_position.y),
            )
        )
        marker.pose.position.z = 0.4
        map_heading = self.current_heading + math.radians(
            self.mission_rotation_deg
        )
        marker.pose.orientation.z = math.sin(map_heading / 2.0)
        marker.pose.orientation.w = math.cos(map_heading / 2.0)
        marker.scale.x = 1.8 * self.drone_scale
        marker.scale.y = 0.7 * self.drone_scale
        marker.scale.z = 0.5 * self.drone_scale
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.98
        return marker

    def _make_marker(
        self,
        namespace: str,
        marker_id: int,
        marker_type: int,
    ) -> Marker:
        """Create a marker with common header fields filled in."""

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def _make_point_from_mission_xy(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Point:
        """Build a Point from mission-local XY after map rotation."""

        map_x, map_y = self._mission_to_map_xy(x, y)
        return Point(x=map_x, y=map_y, z=z)

    def _nearest_active_person(self) -> SceneObject | None:
        """Return the nearest non-rescued person to the drone."""

        active_people = [
            scene_object
            for scene_object in self.person_objects
            if scene_object.name not in self.rescued_person_names
        ]
        if not active_people:
            return None
        if self.vehicle_local_position is None:
            return active_people[0]

        vehicle_x, vehicle_y = self._mission_to_map_xy(
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
        )
        return min(
            active_people,
            key=lambda scene_object: math.hypot(
                scene_object.x - vehicle_x,
                scene_object.y - vehicle_y,
            ),
        )

    def _teleport_person_out_of_scene(self, scene_object: SceneObject) -> bool:
        """Move a rescued person model to a remote holding location."""

        world_name = self.teleport_world_name or Path(
            self.world_sdf_path
        ).stem
        spacing = max(self.rescued_person_teleport_spacing, 0.0)
        offset = float(self.rescued_person_count) * spacing
        request = (
            f'name: "{scene_object.name}" '
            f'position {{ x: {self.rescued_person_teleport_x + offset} '
            f'y: {self.rescued_person_teleport_y} '
            f'z: {self.rescued_person_teleport_z} }} '
            'orientation { x: 0 y: 0 z: 0 w: 1 }'
        )
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
                f'Failed to teleport {scene_object.name}: {error}'
            )
            return False

        if result.returncode != 0:
            error_text = result.stderr.strip() or result.stdout.strip()
            self.get_logger().warning(
                'Gazebo set_pose failed for '
                f'{scene_object.name}: {error_text}'
            )
            return False

        self.rescued_person_count += 1
        return True


def main(args=None):
    """Run the scene map publisher node."""

    rclpy.init(args=args)
    node = SceneMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
