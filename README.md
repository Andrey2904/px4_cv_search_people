# PX4 SITL ArUco Detection and Person Search for `gz_x500_mono_cam`

This workspace now contains ROS 2 Python tools for the PX4 Gazebo GZ monocular camera.

What it does:
- uses real camera images plus `CameraInfo`, not ground truth;
- detects wall or facade markers from the image with `cv2.aruco`;
- runs object detection to find people from the drone camera;
- estimates pose with `cv2.solvePnP`;
- publishes marker poses in the camera frame;
- additionally publishes poses in `base_link` and `world` when matching TF is available;
- publishes an annotated debug image;
- overlays the latest person detection directly on the camera viewer;
- can auto-start `ros_gz_bridge` after auto-discovering the real Gazebo camera topics.

## Auto-discovered camera topics

The launch file polls `gz topic -l` and selects the first matching topic pair for `x500_mono_cam(_N)?`.

On this machine, the expected raw Gazebo topics are:
- `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`
- `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info`

If you run another world, for example `my_world_2`, the launch file still discovers the real topics at runtime instead of hardcoding the world name.

## Main files

- `src/offboard_takeoff/offboard_takeoff/aruco_detector.py`
- `src/offboard_takeoff/offboard_takeoff/yolo_detector.py`
- `src/offboard_takeoff/offboard_takeoff/camera_viewer.py`
- `src/offboard_takeoff/launch/aruco_detection.launch.py`
- `src/offboard_takeoff/config/aruco_detector.yaml`
- `src/offboard_takeoff/config/yolo_detector.yaml`

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

## Run person detection with your YOLO weights

The detector node supports both exported `ONNX` models and trained
Ultralytics `*.pt` checkpoints.

If `yolo_model_path` is left empty, the node now first tries your trained
`yolo12n` checkpoint in `/home/dron/.gz/models/yolo12n_people_package/...`
and then falls back to the newest local model from `models/yolo_runs`.

For a fine-tuned YOLO checkpoint:

```bash
ros2 launch offboard_takeoff aruco_detection.launch.py \
  start_yolo:=true \
  detector_architecture:=yolo \
  yolo_model_path:=/home/dron/.gz/models/yolo12n_people_package/runs/yolo12n_people_v1_safe2/weights/best.pt \
  yolo_inference_backend:=ultralytics \
  yolo_input_width:=960 \
  yolo_input_height:=960
```

If you prefer ONNX, first export the checkpoint:

```bash
python3 tools/export_yolo_onnx.py \
  --weights /home/dron/.gz/models/yolo12n_people_package/runs/yolo12n_people_v1_safe2/weights/best.pt
```

Important runtime note:
- `*.pt` inference needs `ultralytics` installed.
- `*.onnx` inference needs `onnxruntime` or OpenCV DNN.

## Person response logic

The `offboard_takeoff` mission node now uses a simple scripted response after
YOLO detects a person:

1. the person bbox must stay visible for `1.0` second to confirm the target;
2. if detection drops during this confirmation window, the timer resets and the
   next confirmation starts from zero;
3. after confirmation the drone stops briefly to stabilize;
4. it memorizes the current bbox height and flies forward;
5. forward motion continues even if detection drops temporarily;
6. the drone stops only after YOLO sees the person again and the new bbox
   height is about `2x` the memorized height;
7. it then hovers for `5` seconds and returns home.

The key mission parameters for this behavior live in
[`node.py`](/home/dron/px4_offboard_clean_ws/src/offboard_takeoff/offboard_takeoff/node.py):
- `follow_detection_confirm_sec`: how long the bbox must stay visible before engagement.
- `follow_stop_before_approach_sec`: how long to hover before starting the forward move.
- `follow_bbox_growth_target`: bbox growth ratio used as the stop condition.
- `follow_forward_speed`: forward speed used while approaching the target.
- `follow_hover_after_approach_sec`: hover time before return-to-home starts.

The mission node also prints explicit logs when:
- the first person bbox is received;
- target confirmation starts;
- person follow engages;
- the bbox growth stop condition is reached.

## Run person detection with DINO

The detector node supports two detector architectures:
- `yolo` for Ultralytics `*.pt` checkpoints and classic YOLO ONNX exports;
- `dino` for DETR or DINO-style exports with either `[scores, labels, boxes]` or `[class_logits, pred_boxes]`.

To search for people in rubble from the drone camera, start the detector with a DINO-compatible ONNX model:

```bash
ros2 launch offboard_takeoff aruco_detection.launch.py \
  start_yolo:=true \
  detector_architecture:=dino \
  yolo_model_path:=/absolute/path/to/dino_people_detector.onnx \
  yolo_input_width:=640 \
  yolo_input_height:=640 \
  yolo_confidence_threshold:=0.35 \
  dino_resize_mode:=stretch \
  dino_box_format:=cxcywh \
  dino_logit_activation:=sigmoid
```

Notes for this mode:
- the model must already be exported to ONNX;
- the model must expose a fixed class head that includes `person`;
- the node keeps publishing the same `yolo/*` topics, so the PX4 mission code can reuse them unchanged.

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

Object detector parameters live in:

```bash
src/offboard_takeoff/config/yolo_detector.yaml
```

Most important detector parameters:
- `model_architecture`: `yolo` or `dino`.
- `model_path`: path to the detector file (`.onnx` or `.pt`).
- `target_labels`: mission target classes, usually `person`.
- `input_width` and `input_height`: exported model input size.
- `confidence_threshold`: minimum accepted detection confidence.
- `dino_resize_mode`: `stretch` or `letterbox`.
- `dino_box_format`: `cxcywh` or `xyxy`.
- `dino_logit_activation`: `sigmoid`, `softmax`, or `none`.

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
- `/yolo/target_detected` (`std_msgs/msg/Bool`)
- `/yolo/detection_count` (`std_msgs/msg/Int32`)
- `/yolo/class_ids` (`std_msgs/msg/Int32MultiArray`)
- `/yolo/bounding_boxes` (`std_msgs/msg/Int32MultiArray`)
- `/yolo/target_bbox` (`std_msgs/msg/Int32MultiArray`)
- `/yolo/debug_image` (`sensor_msgs/msg/Image`)

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

```bash
ros2 topic echo /yolo/target_bbox
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
- The person detector publishes the best target bbox as `[confidence_milli, cx, cy, w, h, image_w, image_h]`.
- `camera_viewer` now draws the latest detected person directly on the live camera preview.
- Pose is recovered purely from image corners and camera intrinsics.
- Camera frame is taken from `CameraInfo.header.frame_id` when available, otherwise inferred from the Gazebo topic path.
- The detector intentionally avoids `cv_bridge`, so it stays usable on ROS 2 Humble setups where `cv_bridge` is still built against NumPy 1.x.
- The debug stream is disabled by default for performance; when enabled, it is throttled and downscaled so `rqt_image_view` is less laggy than the full raw camera stream.
- `base_link` and `world` outputs stay empty until a valid TF chain from the camera frame exists.
