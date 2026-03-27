# PX4 SITL ArUco Detection for `gz_x500_mono_cam`

This workspace now contains a ROS 2 Python ArUco detector for the PX4 Gazebo GZ monocular camera.

What it does:
- uses real camera images plus `CameraInfo`, not ground truth;
- detects wall or facade markers from the image with `cv2.aruco`;
- estimates pose with `cv2.solvePnP`;
- publishes marker poses in the camera frame;
- additionally publishes poses in `base_link` and `world` when matching TF is available;
- publishes an annotated debug image;
- can auto-start `ros_gz_bridge` after auto-discovering the real Gazebo camera topics.

## Auto-discovered camera topics

The launch file polls `gz topic -l` and selects the first matching topic pair for `x500_mono_cam(_N)?`.

On this machine, the expected raw Gazebo topics are:
- `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`
- `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info`

If you run another world, for example `my_world_2`, the launch file still discovers the real topics at runtime instead of hardcoding the world name.

## Files added

- `src/offboard_takeoff/offboard_takeoff/aruco_detector.py`
- `src/offboard_takeoff/launch/aruco_detection.launch.py`
- `src/offboard_takeoff/config/aruco_detector.yaml`

## Build

```bash
cd ~/px4_offboard_clean_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select offboard_takeoff
source install/setup.bash
```

## Run PX4 SITL + Gazebo GZ

Terminal 1:

```bash
cd ~/PX4-Autopilot
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500_mono_cam
```

Terminal 2:

```bash
MicroXRCEAgent udp4 -p 8888
```

## Run ArUco detection

Terminal 3:

```bash
cd ~/px4_offboard_clean_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch offboard_takeoff aruco_detection.launch.py
```

This launch file:
- finds the real Gazebo `image` and `camera_info` topics;
- starts `ros_gz_bridge` for image, camera info, and `/clock`;
- passes the discovered topics and frame into the detector.

## Useful launch overrides

If the bridge is already running:

```bash
ros2 launch offboard_takeoff aruco_detection.launch.py enable_bridge:=false
```

If you want to override the model regex:

```bash
ros2 launch offboard_takeoff aruco_detection.launch.py model_regex:=x500_mono_cam(_[0-9]+)?
```

If you want to force manual topics:

```bash
ros2 launch offboard_takeoff aruco_detection.launch.py \
  image_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image \
  camera_info_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info \
  camera_frame:=camera_link
```

## Detector parameters

Default parameters live in:

```bash
src/offboard_takeoff/config/aruco_detector.yaml
```

Most important parameters:
- `marker_size_m`: physical marker size in meters.
- `aruco_dictionary`: OpenCV dictionary name such as `DICT_4X4_50`.
- `processing_max_rate_hz`: throttle heavy CV processing to keep Gazebo, QGC, and ROS responsive.
- `detection_hold_sec`: keep the last seen marker ID alive for a short time after a brief detection.
- `publish_debug_image`: enable annotated debug output. Disabled by default for performance.
- `debug_image_max_rate_hz`: throttle debug image publishing to keep viewers responsive.
- `debug_image_scale`: downscale the debug image before publishing it.
- `base_frame_hints` and `world_frame_hints`: preferred TF targets.

## Published topics

- `/aruco/debug_image` (`sensor_msgs/msg/Image`)
- `/aruco/marker_detected` (`std_msgs/msg/Bool`)
- `/aruco/detection_count` (`std_msgs/msg/Int32`)
- `/aruco/marker_ids` (`std_msgs/msg/Int32MultiArray`)
- `/aruco/poses/camera` (`geometry_msgs/msg/PoseArray`)
- `/aruco/markers/camera` (`visualization_msgs/msg/MarkerArray`)
- `/aruco/poses/base_link` (`geometry_msgs/msg/PoseArray`, only when TF exists)
- `/aruco/markers/base_link` (`visualization_msgs/msg/MarkerArray`, only when TF exists)
- `/aruco/poses/world` (`geometry_msgs/msg/PoseArray`, only when TF exists)
- `/aruco/markers/world` (`visualization_msgs/msg/MarkerArray`, only when TF exists)

Marker IDs are visible in the debug image overlay and in the `MarkerArray` text labels.

## Quick inspection

Debug image (off by default for performance):

```bash
ros2 run rqt_image_view rqt_image_view /aruco/debug_image
```

Fast terminal checks without video:

```bash
ros2 topic echo /aruco/marker_detected
```

```bash
ros2 topic echo /aruco/detection_count
```

```bash
ros2 topic echo /aruco/marker_ids
```

Pose output:

```bash
ros2 topic echo /aruco/poses/camera
```

RViz markers:

```bash
rviz2
```

Then add `MarkerArray` displays for `/aruco/markers/camera`, and if TF exists, also `/aruco/markers/base_link` or `/aruco/markers/world`.

## Notes

- The detector is designed for markers mounted on walls, houses, or any other plane seen by the forward camera.
- Pose is recovered purely from image corners and camera intrinsics.
- Camera frame is taken from `CameraInfo.header.frame_id` when available, otherwise inferred from the Gazebo topic path.
- The detector intentionally avoids `cv_bridge`, so it stays usable on ROS 2 Humble setups where `cv_bridge` is still built against NumPy 1.x.
- The debug stream is disabled by default for performance; when enabled, it is throttled and downscaled so `rqt_image_view` is less laggy than the full raw camera stream.
- `base_link` and `world` outputs stay empty until a valid TF chain from the camera frame exists.
