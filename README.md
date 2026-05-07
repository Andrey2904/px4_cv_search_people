# PX4 SITL Person Search with YOLO12s for `gz_x500_mono_cam`

This workspace contains ROS 2 Python tools for a PX4 Gazebo GZ quadcopter
used in a search-and-rescue style simulation scenario.

What the current project does:
- runs a PX4 offboard mission in ROS 2;
- flies a snake-style search route over the target area;
- receives real camera images from the drone through `ros_gz_bridge`;
- detects people from the onboard camera with a fine-tuned `YOLO12s`;
- publishes the best person bounding box to ROS 2 topics;
- overlays the latest person detection in a relay viewer;
- performs a scripted visual response after target confirmation;
- can teleport an evacuated person out of the Gazebo scene;
- continues the snake search for other people after evacuation.

## Main packages

- `src/offboard_takeoff`: mission logic, launch files, viewer, YOLO detector.
- `src/px4_msgs`: PX4 ROS 2 message definitions used by the mission node.

## Main files

- `src/offboard_takeoff/offboard_takeoff/node.py`
- `src/offboard_takeoff/offboard_takeoff/mission.py`
- `src/offboard_takeoff/offboard_takeoff/navigation.py`
- `src/offboard_takeoff/offboard_takeoff/yolo_detector.py`
- `src/offboard_takeoff/offboard_takeoff/camera_viewer.py`
- `src/offboard_takeoff/launch/person_detection.launch.py`
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

## Run the camera bridge and YOLO detector

Terminal 3:

```bash
cd ~/px4_offboard_clean_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch offboard_takeoff person_detection.launch.py \
  start_yolo:=true \
  detector_architecture:=yolo \
  yolo_model_path:=/home/dron/.gz/models/yolo12n_people_package/runs/yolo12s_people_e30_b62/weights/best.pt \
  yolo_inference_backend:=ultralytics \
  yolo_input_width:=960 \
  yolo_input_height:=960
```

This launch file:
- auto-discovers the real Gazebo camera topics;
- starts `ros_gz_bridge` for image, camera info, and `/clock`;
- starts the camera relay viewer;
- optionally starts the detector node.

If the detector model path is empty, the node first tries the trained
`YOLO12s` checkpoint in `/home/dron/.gz/models/yolo12n_people_package/...`
and then falls back to the newest local model from `models/yolo_runs`.

## Mission logic

The offboard mission node is implemented as an explicit state machine:

- `INIT`
- `WAIT_FOR_PX4`
- `ARMING`
- `TAKEOFF`
- `SEARCH`
- `HOLD`
- `FOLLOW_PERSON`
- `RETURN_HOME`
- `LAND`
- `FAILSAFE`
- `FINISHED`

The default mission:
- saves the home position;
- climbs to the working altitude;
- flies a snake-style search route across the area;
- waits for a confirmed person detection;
- performs a visual approach sequence;
- publishes a `mission/person_rescued` event;
- teleports the nearest person model out of the Gazebo world;
- resumes the search route for remaining people;
- returns home and lands after the route finishes.

## Person response logic

After `YOLO12s` detects a person, the mission node runs the following logic:

1. the person bbox must remain visible for `follow_detection_confirm_sec`;
2. if the detection drops for too long, confirmation resets;
3. after confirmation the drone briefly hovers to stabilize;
4. it aligns yaw toward the target using the horizontal bbox error;
5. it memorizes the initial bbox height;
6. it flies forward while the bbox grows in the image;
7. once the target appears large enough, the drone holds position;
8. it marks the person as evacuated;
9. the scene map teleports the nearest person model away with `gz service`;
10. the drone ignores detections briefly and resumes the search route.

Key parameters live in:
- `src/offboard_takeoff/offboard_takeoff/node.py`
- `src/offboard_takeoff/config/yolo_detector.yaml`

Most important mission parameters:
- `resume_search_after_rescue`
- `post_rescue_detection_cooldown_sec`
- `follow_detection_confirm_sec`
- `follow_detection_gap_tolerance_sec`
- `follow_stop_before_approach_sec`
- `follow_bbox_growth_target`
- `follow_forward_speed`
- `follow_hover_after_approach_sec`

Scene-map rescue parameters:
- `teleport_rescued_people`
- `hide_rescued_people`
- `teleport_world_name`
- `rescued_person_teleport_x`
- `rescued_person_teleport_y`
- `rescued_person_teleport_z`

## Detector options

The detector node supports YOLO inference with Ultralytics `*.pt` checkpoints
and classic YOLO ONNX exports.

Important runtime note:
- `*.pt` inference needs `ultralytics`;
- `*.onnx` inference uses `onnxruntime` when available and falls back to OpenCV DNN.

## Useful launch overrides

If the bridge is already running:

```bash
ros2 launch offboard_takeoff person_detection.launch.py enable_bridge:=false
```

If you want to force manual topics:

```bash
ros2 launch offboard_takeoff person_detection.launch.py \
  image_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image \
  camera_info_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info
```

## Published topics

- `/camera/image_raw`
- `/camera/camera_info`
- `/yolo/target_detected`
- `/yolo/detection_count`
- `/yolo/class_ids`
- `/yolo/bounding_boxes`
- `/yolo/target_bbox`
- `/yolo/class_labels`
- `/yolo/debug_image`

The topic `/yolo/target_bbox` is the key interface between computer vision and
the offboard mission logic.

## YOLO fine-tuning

Training assets are stored in `training/yolo` and `tools`.

Expected workflow:
1. audit the dataset with `tools/audit_yolo_dataset.py`;
2. train with `scripts/train_yolo_gpu.sh`;
3. export the best checkpoint with `tools/export_yolo_onnx.py`.

Useful files:
- `training/yolo/README.md`
- `tools/audit_yolo_dataset.py`
- `tools/train_yolo.py`
- `tools/export_yolo_onnx.py`
- `tools/predict_video_yolo.py`
