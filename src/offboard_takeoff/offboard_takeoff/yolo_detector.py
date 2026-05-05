"""ROS 2 object detector node with YOLO and DINO-style ONNX backends."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


def _extend_sys_path_with_local_venvs():
    """Add local workspace virtualenv site-packages when available."""

    current_path = Path(__file__).resolve()
    candidate_roots = []
    for base in [current_path.parent, *current_path.parents]:
        candidate_roots.extend(
            (
                base / '.venv-yolo' / 'lib',
                base / 'models' / '.venv' / 'lib',
            )
        )

    for candidate in candidate_roots:
        if not candidate.is_dir():
            continue
        for python_dir in sorted(candidate.glob('python*/site-packages')):
            path_string = str(python_dir)
            if path_string not in sys.path:
                sys.path.insert(0, path_string)


_extend_sys_path_with_local_venvs()

try:
    import onnxruntime as ort
except ImportError:
    ort = None

try:
    from ultralytics import YOLO as UltralyticsYOLO
except ImportError:
    UltralyticsYOLO = None

try:
    import torch
except ImportError:
    torch = None


COCO_CLASS_NAMES = (
    'person',
    'bicycle',
    'car',
    'motorcycle',
    'airplane',
    'bus',
    'train',
    'truck',
    'boat',
    'traffic light',
    'fire hydrant',
    'stop sign',
    'parking meter',
    'bench',
    'bird',
    'cat',
    'dog',
    'horse',
    'sheep',
    'cow',
    'elephant',
    'bear',
    'zebra',
    'giraffe',
    'backpack',
    'umbrella',
    'handbag',
    'tie',
    'suitcase',
    'frisbee',
    'skis',
    'snowboard',
    'sports ball',
    'kite',
    'baseball bat',
    'baseball glove',
    'skateboard',
    'surfboard',
    'tennis racket',
    'bottle',
    'wine glass',
    'cup',
    'fork',
    'knife',
    'spoon',
    'bowl',
    'banana',
    'apple',
    'sandwich',
    'orange',
    'broccoli',
    'carrot',
    'hot dog',
    'pizza',
    'donut',
    'cake',
    'chair',
    'couch',
    'potted plant',
    'bed',
    'dining table',
    'toilet',
    'tv',
    'laptop',
    'mouse',
    'remote',
    'keyboard',
    'cell phone',
    'microwave',
    'oven',
    'toaster',
    'sink',
    'refrigerator',
    'book',
    'clock',
    'vase',
    'scissors',
    'teddy bear',
    'hair drier',
    'toothbrush',
)


DEFAULT_EXTERNAL_MODEL_CANDIDATES = (
    Path('/home/dron/.gz/models/yolo12n_people_package/runs')
    / 'yolo12s_people_e30_b62'
    / 'weights'
    / 'best.pt',
    Path('/home/dron/.gz/models/yolo12n_people_package/runs')
    / 'yolo12s_people_e30_b62'
    / 'weights'
    / 'last.pt',
    Path('/home/dron/.gz/models/yolo12n_people_package/runs')
    / 'yolo12n_people_v1_safe2'
    / 'weights'
    / 'best.pt',
    Path('/home/dron/.gz/models/yolo12n_people_package/runs')
    / 'yolo12n_people_v1_safe2'
    / 'weights'
    / 'last.pt',
)


@dataclass(frozen=True)
class Detection:
    """Single object detection in image coordinates."""

    class_id: int
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int


@dataclass(frozen=True)
class LetterboxResult:
    """Image resized with preserved aspect ratio and padding metadata."""

    image: np.ndarray
    scale: float
    pad_x: float
    pad_y: float


@dataclass(frozen=True)
class ResizeResult:
    """Image resized to the model input with scale metadata."""

    image: np.ndarray
    scale_x: float
    scale_y: float


class YoloDetector(Node):
    """Detect people and other classes from a ROS image topic."""

    def __init__(self):
        super().__init__('yolo_detector')

        self._declare_parameters()
        self._load_parameters()

        self.inference_backend = ''
        self.session = None
        self.session_input_name = ''
        self.session_output_names: list[str] = []
        self.net = None
        self.output_names: list[str] = []
        self.ultralytics_model = None
        self.ultralytics_device = 'cpu'
        self.ultralytics_use_half = False
        self.resolved_model_path: Path | None = None
        self._initialize_inference_backend()
        self.class_names = self._load_class_names()
        self.target_labels_set = {
            label.strip().lower()
            for label in self.target_labels
            if label.strip()
        }

        self.last_processed_frame_ns = 0
        self.last_debug_image_pub_ns = 0
        self.last_detection_state = False
        self.last_seen_target_ns = 0
        self.output_shape_logged = False
        self.last_candidate_log_ns = 0
        self.last_target_summary_log_ns = 0
        self.forward_error_logged = False
        self.last_fps_timestamp_ns = 0
        self.smoothed_fps = 0.0

        self.debug_image_pub = self.create_publisher(
            Image,
            'yolo/debug_image',
            qos_profile_sensor_data,
        )
        self.target_detected_pub = self.create_publisher(
            Bool,
            'yolo/target_detected',
            10,
        )
        self.detection_count_pub = self.create_publisher(
            Int32,
            'yolo/detection_count',
            10,
        )
        self.class_ids_pub = self.create_publisher(
            Int32MultiArray,
            'yolo/class_ids',
            10,
        )
        self.bounding_boxes_pub = self.create_publisher(
            Int32MultiArray,
            'yolo/bounding_boxes',
            10,
        )
        self.target_bbox_pub = self.create_publisher(
            Int32MultiArray,
            'yolo/target_bbox',
            10,
        )
        self.class_labels_pub = self.create_publisher(
            String,
            'yolo/class_labels',
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Detector started: '
            f'architecture={self.model_architecture}, '
            f'image_topic={self.image_topic}, '
            f'model_path={self.model_path}, '
            f'input_size={self.input_width}x{self.input_height}, '
            f'confidence_threshold={self.confidence_threshold:.2f}, '
            f'nms_threshold={self.nms_threshold:.2f}, '
            f'target_labels={list(self.target_labels_set) or ["<any>"]}'
        )

    def _declare_parameters(self):
        """Declare ROS parameters."""

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_architecture', 'yolo')
        self.declare_parameter('inference_backend', 'auto')
        self.declare_parameter('prefer_gpu', True)
        self.declare_parameter('class_names', list(COCO_CLASS_NAMES))
        self.declare_parameter('target_labels', ['person'])
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('processing_max_rate_hz', 3.0)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_max_rate_hz', 2.0)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('debug_view_scale', 0.75)
        self.declare_parameter('show_fps_overlay', True)
        self.declare_parameter('detection_hold_sec', 0.75)
        self.declare_parameter('swap_rb', True)
        self.declare_parameter('diagnostic_log_period_sec', 0.0)
        self.declare_parameter('target_summary_log_period_sec', 1.5)
        self.declare_parameter('dino_resize_mode', 'stretch')
        self.declare_parameter('dino_box_format', 'cxcywh')
        self.declare_parameter('dino_logit_activation', 'sigmoid')
        self.declare_parameter(
            'dino_normalize_mean',
            [0.485, 0.456, 0.406],
        )
        self.declare_parameter(
            'dino_normalize_std',
            [0.229, 0.224, 0.225],
        )
        self.declare_parameter('dino_label_offset', 0)

    def _load_parameters(self):
        """Load ROS parameters into node fields."""

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.model_path = str(self.get_parameter('model_path').value).strip()
        self.model_architecture = str(
            self.get_parameter('model_architecture').value
        ).strip().lower()
        self.requested_inference_backend = str(
            self.get_parameter('inference_backend').value
        ).strip().lower()
        self.prefer_gpu = bool(self.get_parameter('prefer_gpu').value)
        self.target_labels = list(self.get_parameter('target_labels').value)
        self.input_width = max(
            int(self.get_parameter('input_width').value),
            32,
        )
        self.input_height = max(
            int(self.get_parameter('input_height').value),
            32,
        )
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.score_threshold = float(
            self.get_parameter('score_threshold').value
        )
        self.nms_threshold = float(
            self.get_parameter('nms_threshold').value
        )
        self.processing_max_rate_hz = float(
            self.get_parameter('processing_max_rate_hz').value
        )
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )
        self.debug_image_max_rate_hz = float(
            self.get_parameter('debug_image_max_rate_hz').value
        )
        self.show_debug_window = bool(
            self.get_parameter('show_debug_window').value
        )
        self.debug_view_scale = float(
            self.get_parameter('debug_view_scale').value
        )
        self.show_fps_overlay = bool(
            self.get_parameter('show_fps_overlay').value
        )
        self.detection_hold_sec = float(
            self.get_parameter('detection_hold_sec').value
        )
        self.swap_rb = bool(self.get_parameter('swap_rb').value)
        self.diagnostic_log_period_sec = float(
            self.get_parameter('diagnostic_log_period_sec').value
        )
        self.target_summary_log_period_sec = float(
            self.get_parameter('target_summary_log_period_sec').value
        )
        self.dino_resize_mode = str(
            self.get_parameter('dino_resize_mode').value
        ).strip().lower()
        self.dino_box_format = str(
            self.get_parameter('dino_box_format').value
        ).strip().lower()
        self.dino_logit_activation = str(
            self.get_parameter('dino_logit_activation').value
        ).strip().lower()
        self.dino_normalize_mean = self._load_float_triplet(
            'dino_normalize_mean'
        )
        self.dino_normalize_std = self._load_float_triplet(
            'dino_normalize_std'
        )
        self.dino_label_offset = int(
            self.get_parameter('dino_label_offset').value
        )

        if self.model_architecture not in {'yolo', 'dino'}:
            raise ValueError(
                'Unsupported model_architecture. Use "yolo" or "dino".'
            )
        if self.dino_resize_mode not in {'stretch', 'letterbox'}:
            raise ValueError(
                'Unsupported dino_resize_mode. '
                'Use "stretch" or "letterbox".'
            )
        if self.dino_box_format not in {'cxcywh', 'xyxy'}:
            raise ValueError(
                'Unsupported dino_box_format. Use "cxcywh" or "xyxy".'
            )
        if self.dino_logit_activation not in {'sigmoid', 'softmax', 'none'}:
            raise ValueError(
                'Unsupported dino_logit_activation. '
                'Use "sigmoid", "softmax", or "none".'
            )

    def _load_float_triplet(self, parameter_name: str) -> tuple[float, float, float]:
        """Parse a float triplet parameter."""

        values = list(self.get_parameter(parameter_name).value)
        if len(values) != 3:
            raise ValueError(
                f'Parameter "{parameter_name}" must have exactly 3 values.'
            )
        return tuple(float(value) for value in values)

    def _resolve_model_path(self, model_path: str) -> Path:
        """Resolve and validate the detector model path."""

        if not model_path:
            auto_discovered = self._find_default_model_path()
            if auto_discovered is None:
                raise ValueError(
                    'Parameter "model_path" is empty and no local detector '
                    'weights were auto-discovered. Provide a path to an ONNX '
                    'file or a trained YOLO .pt checkpoint.'
                )
            self.get_logger().info(
                f'Auto-selected detector model: {auto_discovered}'
            )
            resolved_path = auto_discovered
        else:
            resolved_path = Path(model_path).expanduser()

        if not resolved_path.is_file():
            raise FileNotFoundError(
                f'Detector model file was not found: {resolved_path}'
            )
        return resolved_path

    def _find_default_model_path(self) -> Path | None:
        """Pick the preferred detector checkpoint or the newest fallback."""

        workspace_root = Path(__file__).resolve().parents[3]
        for preferred_path in DEFAULT_EXTERNAL_MODEL_CANDIDATES:
            if preferred_path.is_file():
                return preferred_path

        search_roots_and_patterns = (
            (
                workspace_root,
                (
                    'models/yolo_runs/**/*.onnx',
                    'models/**/*.onnx',
                    'models/yolo_runs/**/weights/best.pt',
                    'models/yolo_runs/**/weights/last.pt',
                ),
            ),
            (
                Path('/home/dron/.gz/models'),
                (
                    'yolo12n_people_package/runs/**/weights/best.pt',
                    'yolo12n_people_package/runs/**/weights/last.pt',
                    '**/*.onnx',
                ),
            ),
        )

        candidates: list[Path] = []
        for root, patterns in search_roots_and_patterns:
            if not root.is_dir():
                continue
            for pattern in patterns:
                candidates.extend(
                    path
                    for path in root.glob(pattern)
                    if path.is_file()
                )

        if not candidates:
            return None

        return max(candidates, key=lambda path: path.stat().st_mtime)

    def _initialize_inference_backend(self):
        """Initialize the requested inference backend with fallback."""

        resolved_path = self._resolve_model_path(self.model_path)
        backend = self.requested_inference_backend

        if backend not in {'auto', 'onnxruntime', 'opencv', 'ultralytics'}:
            raise ValueError(
                'Unsupported inference_backend. '
                'Use one of: auto, onnxruntime, opencv, ultralytics.'
            )

        self.resolved_model_path = resolved_path
        if resolved_path.suffix.lower() == '.pt':
            if backend in {'auto', 'ultralytics'}:
                self._initialize_ultralytics(resolved_path)
                return
            raise ValueError(
                'PyTorch checkpoints (.pt) require '
                'inference_backend:=ultralytics or auto.'
            )

        if backend in {'auto', 'onnxruntime'}:
            if self._try_initialize_onnxruntime(resolved_path):
                return
            if backend == 'onnxruntime':
                raise RuntimeError(
                    'inference_backend:=onnxruntime was requested, but '
                    'ONNX Runtime could not be initialized.'
                )

        self._initialize_opencv_dnn(resolved_path)

    def _initialize_ultralytics(self, model_path: Path):
        """Initialize Ultralytics for native PyTorch YOLO checkpoints."""

        if UltralyticsYOLO is None:
            raise ImportError(
                'Ultralytics is required to run .pt checkpoints. '
                'Install training/yolo/requirements.txt or export the model '
                'to ONNX and use inference_backend:=onnxruntime.'
            )

        if (
            self.prefer_gpu
            and torch is not None
            and torch.cuda.is_available()
        ):
            self.ultralytics_device = 'cuda:0'
            self.ultralytics_use_half = True
        else:
            self.ultralytics_device = 'cpu'
            self.ultralytics_use_half = False

        self.ultralytics_model = UltralyticsYOLO(str(model_path))
        self.inference_backend = 'ultralytics'
        self.get_logger().info(
            'Inference backend: Ultralytics '
            f'model={model_path.name}, '
            f'device={self.ultralytics_device}, '
            f'half={self.ultralytics_use_half}'
        )

    def _try_initialize_onnxruntime(self, model_path: Path) -> bool:
        """Try to initialize ONNX Runtime with CUDA or CPU providers."""

        if ort is None:
            self.get_logger().warning(
                'onnxruntime is not installed in the current Python '
                'environment, falling back to OpenCV DNN.'
            )
            return False

        preload_dlls = getattr(ort, 'preload_dlls', None)
        if callable(preload_dlls):
            try:
                preload_dlls()
            except Exception as error:
                self.get_logger().warning(
                    'onnxruntime preload_dlls() failed, continuing without '
                    f'preloaded CUDA libraries: {error}'
                )

        available_providers = ort.get_available_providers()
        providers: list[str] = []
        if self.prefer_gpu and 'CUDAExecutionProvider' in available_providers:
            providers.append('CUDAExecutionProvider')
        providers.append('CPUExecutionProvider')

        try:
            self.session = ort.InferenceSession(
                str(model_path),
                providers=providers,
            )
        except Exception as error:
            self.get_logger().warning(
                'Failed to initialize ONNX Runtime, falling back to OpenCV '
                f'DNN. Reason: {error}'
            )
            self.session = None
            return False

        self.session_input_name = self.session.get_inputs()[0].name
        self.session_output_names = [
            output.name for output in self.session.get_outputs()
        ]
        self.inference_backend = 'onnxruntime'
        self.get_logger().info(
            'Inference backend: ONNX Runtime '
            f'providers={self.session.get_providers()}'
        )
        return True

    def _initialize_opencv_dnn(self, model_path: Path):
        """Initialize OpenCV DNN as a fallback backend."""

        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.output_names = self.net.getUnconnectedOutLayersNames()
        self.inference_backend = 'opencv'
        self.get_logger().info('Inference backend: OpenCV DNN providers=[CPU]')

    def _load_class_names(self) -> list[str]:
        """Resolve the class list from parameters."""

        parameter_value = list(self.get_parameter('class_names').value)
        class_names = [str(name).strip() for name in parameter_value if str(name).strip()]
        if (
            self.inference_backend == 'ultralytics'
            and self.ultralytics_model is not None
            and class_names == list(COCO_CLASS_NAMES)
        ):
            model_names = getattr(self.ultralytics_model, 'names', None)
            if isinstance(model_names, dict):
                return [
                    str(model_names[index]).strip()
                    for index in sorted(model_names)
                ]
            if isinstance(model_names, (list, tuple)):
                return [
                    str(name).strip()
                    for name in model_names
                    if str(name).strip()
                ]
        if not class_names:
            return list(COCO_CLASS_NAMES)
        return class_names

    def _image_callback(self, msg: Image):
        """Run detection on the incoming ROS image."""

        if not self._should_process_frame():
            return

        frame = self._image_to_bgr(msg)
        if frame is None:
            return

        detections = self._run_inference(frame)
        self._publish_detection_status(
            detections=detections,
            frame_width=frame.shape[1],
            frame_height=frame.shape[0],
        )
        self._update_fps()

        if not self._should_publish_debug_image():
            if self.show_debug_window:
                debug_frame = self._draw_detections(frame.copy(), detections)
                debug_frame = self._prepare_debug_frame(debug_frame)
                cv2.imshow('Detector Debug View', debug_frame)
                cv2.waitKey(1)
            return

        debug_frame = self._draw_detections(frame.copy(), detections)
        prepared_debug_frame = self._prepare_debug_frame(debug_frame)
        if self.show_debug_window:
            cv2.imshow('Detector Debug View', prepared_debug_frame)
            cv2.waitKey(1)
        debug_msg = self._bgr_to_image_message(
            frame=prepared_debug_frame,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
        )
        self.debug_image_pub.publish(debug_msg)

    def _run_inference(self, frame: np.ndarray) -> list[Detection]:
        """Run model-specific inference and post-processing."""

        if self.inference_backend == 'ultralytics':
            return self._run_ultralytics_inference(frame)
        if self.model_architecture == 'dino':
            return self._run_dino_inference(frame)
        return self._run_yolo_inference(frame)

    def _run_ultralytics_inference(
        self,
        frame: np.ndarray,
    ) -> list[Detection]:
        """Run native Ultralytics inference for .pt YOLO checkpoints."""

        if self.ultralytics_model is None:
            return []

        try:
            target_class_ids = self._target_class_ids_for_ultralytics()
            results = self.ultralytics_model.predict(
                source=frame,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                imgsz=max(self.input_width, self.input_height),
                device=self.ultralytics_device,
                half=self.ultralytics_use_half,
                classes=target_class_ids if target_class_ids else None,
                verbose=False,
            )
        except Exception as error:
            if not self.forward_error_logged:
                self.get_logger().error(
                    f'Ultralytics inference failed: {error}'
                )
                self.forward_error_logged = True
            return []

        detections: list[Detection] = []
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue

            xyxy = getattr(boxes, 'xyxy', None)
            confidences = getattr(boxes, 'conf', None)
            class_ids = getattr(boxes, 'cls', None)
            if xyxy is None or confidences is None or class_ids is None:
                continue

            xyxy_rows = (
                xyxy.cpu().numpy() if hasattr(xyxy, 'cpu')
                else np.asarray(xyxy)
            )
            confidence_rows = (
                confidences.cpu().numpy()
                if hasattr(confidences, 'cpu')
                else np.asarray(confidences)
            )
            class_id_rows = (
                class_ids.cpu().numpy()
                if hasattr(class_ids, 'cpu')
                else np.asarray(class_ids)
            )

            for raw_box, raw_confidence, raw_class_id in zip(
                xyxy_rows,
                confidence_rows,
                class_id_rows,
            ):
                class_id = int(raw_class_id)
                x1 = int(round(raw_box[0]))
                y1 = int(round(raw_box[1]))
                x2 = int(round(raw_box[2]))
                y2 = int(round(raw_box[3]))
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append(
                    Detection(
                        class_id=class_id,
                        label=self._label_for_class_id(class_id),
                        confidence=float(raw_confidence),
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                    )
                )

        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def _target_class_ids_for_ultralytics(self) -> list[int]:
        """Resolve target label names into Ultralytics class IDs."""

        if not self.target_labels_set:
            return []

        target_ids: list[int] = []
        for class_id, class_name in enumerate(self.class_names):
            if class_name.lower() in self.target_labels_set:
                target_ids.append(class_id)
        return target_ids

    def _run_yolo_inference(self, frame: np.ndarray) -> list[Detection]:
        """Run YOLO inference and post-processing."""

        frame_height, frame_width = frame.shape[:2]
        letterbox = self._letterbox(frame)
        blob = cv2.dnn.blobFromImage(
            letterbox.image,
            scalefactor=1.0 / 255.0,
            size=(self.input_width, self.input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=self.swap_rb,
            crop=False,
        )
        outputs = self._forward(blob)
        if outputs is None:
            return []
        predictions = self._normalize_yolo_predictions(outputs)
        if predictions is None or predictions.size == 0:
            return []

        self._log_best_candidates(predictions)

        boxes: list[list[int]] = []
        confidences: list[float] = []
        class_ids: list[int] = []

        for row in predictions:
            class_id, confidence, box = self._decode_yolo_prediction_row(
                row=row,
                letterbox=letterbox,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if class_id is None or box is None:
                continue
            boxes.append(box)
            confidences.append(confidence)
            class_ids.append(class_id)

        return self._detections_from_boxes(
            boxes=boxes,
            confidences=confidences,
            class_ids=class_ids,
            frame_width=frame_width,
            frame_height=frame_height,
            apply_nms=True,
        )

    def _run_dino_inference(self, frame: np.ndarray) -> list[Detection]:
        """Run DINO or DETR-style inference and post-processing."""

        frame_height, frame_width = frame.shape[:2]
        if self.dino_resize_mode == 'letterbox':
            resize_result = self._letterbox(frame)
            resized_image = resize_result.image
        else:
            resize_result = self._resize_to_input(frame)
            resized_image = resize_result.image

        rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
        image_tensor = rgb_image.astype(np.float32) / 255.0
        mean = np.array(self.dino_normalize_mean, dtype=np.float32).reshape(1, 1, 3)
        std = np.array(self.dino_normalize_std, dtype=np.float32).reshape(1, 1, 3)
        image_tensor = (image_tensor - mean) / std
        image_tensor = np.transpose(image_tensor, (2, 0, 1))[np.newaxis, ...]

        outputs = self._forward(image_tensor)
        if outputs is None:
            return []

        detections = self._decode_dino_outputs(
            outputs=outputs,
            resize_result=resize_result,
            frame_width=frame_width,
            frame_height=frame_height,
        )
        if self.diagnostic_log_period_sec > 0.0:
            self._log_detections_snapshot(detections, prefix='DINO')
        return detections

    def _detections_from_boxes(
        self,
        boxes: list[list[int]],
        confidences: list[float],
        class_ids: list[int],
        frame_width: int,
        frame_height: int,
        apply_nms: bool,
    ) -> list[Detection]:
        """Convert raw box lists into sorted Detection objects."""

        if not boxes:
            return []

        kept_indices = range(len(boxes))
        if apply_nms:
            kept_indices = cv2.dnn.NMSBoxes(
                boxes,
                confidences,
                self.confidence_threshold,
                self.nms_threshold,
            )
            if len(kept_indices) == 0:
                return []

        detections: list[Detection] = []
        for raw_index in kept_indices:
            index = int(
                raw_index[0]
                if isinstance(raw_index, (list, tuple, np.ndarray))
                else raw_index
            )
            x, y, width, height = boxes[index]
            x1 = max(x, 0)
            y1 = max(y, 0)
            x2 = min(x + width, frame_width - 1)
            y2 = min(y + height, frame_height - 1)
            if x2 <= x1 or y2 <= y1:
                continue
            class_id = class_ids[index]
            detections.append(
                Detection(
                    class_id=class_id,
                    label=self._label_for_class_id(class_id),
                    confidence=float(confidences[index]),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )

        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def _decode_dino_outputs(
        self,
        outputs: tuple[np.ndarray, ...] | list[np.ndarray],
        resize_result: LetterboxResult | ResizeResult,
        frame_width: int,
        frame_height: int,
    ) -> list[Detection]:
        """Decode common DINO/DETR ONNX output layouts."""

        normalized_outputs = [np.squeeze(np.array(output)) for output in outputs]
        if not normalized_outputs:
            return []

        if not self.output_shape_logged:
            shapes = [tuple(output.shape) for output in normalized_outputs]
            self.get_logger().info(f'DINO raw output shapes: {shapes}')
            self.output_shape_logged = True

        direct = self._decode_dino_direct_outputs(
            normalized_outputs,
            resize_result,
            frame_width,
            frame_height,
        )
        if direct is not None:
            return direct

        logits_boxes = self._decode_dino_logits_and_boxes(
            normalized_outputs,
            resize_result,
            frame_width,
            frame_height,
        )
        if logits_boxes is not None:
            return logits_boxes

        self.get_logger().error(
            'Unsupported DINO output layout. Expected either '
            '[scores, labels, boxes] or [class_logits, pred_boxes].'
        )
        return []

    def _decode_dino_direct_outputs(
        self,
        outputs: list[np.ndarray],
        resize_result: LetterboxResult | ResizeResult,
        frame_width: int,
        frame_height: int,
    ) -> list[Detection] | None:
        """Decode outputs laid out as scores, labels, and boxes."""

        if len(outputs) < 3:
            return None

        labels_array = next(
            (
                output
                for output in outputs
                if output.ndim == 1
                and (
                    np.issubdtype(output.dtype, np.integer)
                    or np.allclose(output, np.round(output), atol=1e-3)
                )
            ),
            None,
        )
        score_array = next(
            (
                output
                for output in outputs
                if output.ndim == 1 and np.issubdtype(output.dtype, np.floating)
            ),
            None,
        )
        box_array = next(
            (
                output
                for output in outputs
                if output.ndim == 2 and output.shape[-1] == 4
            ),
            None,
        )
        if labels_array is None or score_array is None or box_array is None:
            return None
        if len(labels_array) != len(score_array) or len(labels_array) != len(box_array):
            return None

        boxes: list[list[int]] = []
        confidences: list[float] = []
        class_ids: list[int] = []
        for raw_label, raw_score, raw_box in zip(labels_array, score_array, box_array):
            confidence = float(raw_score)
            if confidence < self.confidence_threshold:
                continue
            class_id = int(round(float(raw_label))) + self.dino_label_offset
            if class_id < 0:
                continue
            decoded_box = self._decode_dino_box(
                raw_box,
                resize_result,
                frame_width,
                frame_height,
            )
            if decoded_box is None:
                continue
            boxes.append(decoded_box)
            confidences.append(confidence)
            class_ids.append(class_id)

        return self._detections_from_boxes(
            boxes=boxes,
            confidences=confidences,
            class_ids=class_ids,
            frame_width=frame_width,
            frame_height=frame_height,
            apply_nms=True,
        )

    def _decode_dino_logits_and_boxes(
        self,
        outputs: list[np.ndarray],
        resize_result: LetterboxResult | ResizeResult,
        frame_width: int,
        frame_height: int,
    ) -> list[Detection] | None:
        """Decode outputs laid out as class logits and predicted boxes."""

        box_array = next(
            (
                output
                for output in outputs
                if output.ndim == 2 and output.shape[-1] == 4
            ),
            None,
        )
        logits_candidates = [
            output
            for output in outputs
            if output.ndim == 2
            and output.shape[-1] != 4
            and (
                box_array is None or output.shape[0] == box_array.shape[0]
            )
        ]
        logits_array = (
            max(logits_candidates, key=lambda output: output.shape[-1])
            if logits_candidates
            else None
        )
        if logits_array is None or box_array is None:
            return None
        if logits_array.shape[0] != box_array.shape[0]:
            return None

        if self.dino_logit_activation == 'sigmoid':
            class_scores = 1.0 / (1.0 + np.exp(-logits_array))
        elif self.dino_logit_activation == 'softmax':
            shifted_logits = logits_array - np.max(
                logits_array,
                axis=1,
                keepdims=True,
            )
            class_scores = np.exp(shifted_logits)
            class_scores /= np.sum(class_scores, axis=1, keepdims=True)
        else:
            class_scores = logits_array

        boxes: list[list[int]] = []
        confidences: list[float] = []
        class_ids: list[int] = []

        for logits_row, raw_box in zip(class_scores, box_array):
            class_id = int(np.argmax(logits_row))
            if class_id >= len(self.class_names):
                continue
            confidence = float(logits_row[class_id])
            if confidence < self.confidence_threshold:
                continue
            if confidence < self.score_threshold:
                continue
            decoded_box = self._decode_dino_box(
                raw_box,
                resize_result,
                frame_width,
                frame_height,
            )
            if decoded_box is None:
                continue
            boxes.append(decoded_box)
            confidences.append(confidence)
            class_ids.append(class_id + self.dino_label_offset)

        return self._detections_from_boxes(
            boxes=boxes,
            confidences=confidences,
            class_ids=class_ids,
            frame_width=frame_width,
            frame_height=frame_height,
            apply_nms=True,
        )

    def _decode_dino_box(
        self,
        raw_box: np.ndarray,
        resize_result: LetterboxResult | ResizeResult,
        frame_width: int,
        frame_height: int,
    ) -> list[int] | None:
        """Project a DINO-format box back into the source frame."""

        box = raw_box.astype(np.float32)
        if self.dino_box_format == 'cxcywh':
            cx, cy, width, height = box
            x1 = cx - width / 2.0
            y1 = cy - height / 2.0
            x2 = cx + width / 2.0
            y2 = cy + height / 2.0
        else:
            x1, y1, x2, y2 = box

        if max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 2.0:
            x1 *= self.input_width
            x2 *= self.input_width
            y1 *= self.input_height
            y2 *= self.input_height

        if isinstance(resize_result, LetterboxResult):
            x1 = (x1 - resize_result.pad_x) / resize_result.scale
            x2 = (x2 - resize_result.pad_x) / resize_result.scale
            y1 = (y1 - resize_result.pad_y) / resize_result.scale
            y2 = (y2 - resize_result.pad_y) / resize_result.scale
        else:
            x1 /= resize_result.scale_x
            x2 /= resize_result.scale_x
            y1 /= resize_result.scale_y
            y2 /= resize_result.scale_y

        x1 = max(min(int(round(x1)), frame_width - 1), 0)
        y1 = max(min(int(round(y1)), frame_height - 1), 0)
        x2 = max(min(int(round(x2)), frame_width - 1), 0)
        y2 = max(min(int(round(y2)), frame_height - 1), 0)
        width_px = x2 - x1
        height_px = y2 - y1
        if width_px <= 1 or height_px <= 1:
            return None
        return [x1, y1, width_px, height_px]

    def _log_detections_snapshot(
        self,
        detections: list[Detection],
        prefix: str,
    ):
        """Periodically log the strongest decoded detections."""

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(self.diagnostic_log_period_sec * 1e9)
        if now_ns - self.last_candidate_log_ns < min_period_ns:
            return

        if detections:
            summary = ', '.join(
                f'{detection.label}:{detection.confidence:.3f}'
                for detection in detections[:5]
            )
        else:
            summary = '<none>'
        self.get_logger().info(f'{prefix} top detections: {summary}')
        self.last_candidate_log_ns = now_ns

    def _log_best_candidates(self, predictions: np.ndarray):
        """Periodically log the best raw class candidates for debugging."""

        if self.diagnostic_log_period_sec <= 0.0:
            return

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(self.diagnostic_log_period_sec * 1e9)
        if now_ns - self.last_candidate_log_ns < min_period_ns:
            return

        summaries: list[tuple[float, str]] = []
        for row in predictions[: min(len(predictions), 200)]:
            if row.shape[0] < 4 + len(self.class_names):
                continue

            class_scores = row[4:]
            if row.shape[0] >= 5 + len(self.class_names):
                objectness = float(row[4])
                class_scores = row[5:]
            else:
                objectness = 1.0

            class_id = int(np.argmax(class_scores))
            class_score = float(class_scores[class_id])
            confidence = objectness * class_score
            summaries.append((confidence, self._label_for_class_id(class_id)))

        if not summaries:
            return

        summaries.sort(key=lambda item: item[0], reverse=True)
        top_summaries = ', '.join(
            f'{label}:{confidence:.3f}'
            for confidence, label in summaries[:5]
        )
        self.get_logger().info(f'YOLO top raw candidates: {top_summaries}')
        self.last_candidate_log_ns = now_ns

    def _letterbox(self, frame: np.ndarray) -> LetterboxResult:
        """Resize image with aspect ratio preserved and symmetric padding."""

        frame_height, frame_width = frame.shape[:2]
        scale = min(
            self.input_width / float(frame_width),
            self.input_height / float(frame_height),
        )
        resized_width = max(int(round(frame_width * scale)), 1)
        resized_height = max(int(round(frame_height * scale)), 1)

        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_width = self.input_width - resized_width
        pad_height = self.input_height - resized_height
        pad_left = pad_width // 2
        pad_right = pad_width - pad_left
        pad_top = pad_height // 2
        pad_bottom = pad_height - pad_top

        letterboxed = cv2.copyMakeBorder(
            resized,
            pad_top,
            pad_bottom,
            pad_left,
            pad_right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return LetterboxResult(
            image=letterboxed,
            scale=scale,
            pad_x=float(pad_left),
            pad_y=float(pad_top),
        )

    def _resize_to_input(self, frame: np.ndarray) -> ResizeResult:
        """Resize without padding and track separate x/y scales."""

        frame_height, frame_width = frame.shape[:2]
        resized = cv2.resize(
            frame,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        return ResizeResult(
            image=resized,
            scale_x=self.input_width / float(frame_width),
            scale_y=self.input_height / float(frame_height),
        )

    def _normalize_yolo_predictions(
        self,
        outputs: tuple[np.ndarray, ...] | list[np.ndarray],
    ) -> np.ndarray | None:
        """Convert common YOLO ONNX output layouts to NxF rows."""

        if not outputs:
            return None

        output = np.array(outputs[0])
        if not self.output_shape_logged:
            self.get_logger().info(
                f'YOLO raw output shape: {tuple(output.shape)}'
            )
            self.output_shape_logged = True
        output = np.squeeze(output)

        if output.ndim == 1:
            return None

        if output.ndim != 2:
            return output.reshape(output.shape[-2], output.shape[-1])

        expected_features = len(self.class_names) + 4
        if (
            output.shape[0] == expected_features
            or output.shape[0] == expected_features + 1
        ):
            output = output.transpose()

        return output

    def _prepare_debug_frame(self, frame: np.ndarray) -> np.ndarray:
        """Optionally downscale the debug frame for smoother display."""

        if self.show_fps_overlay:
            self._draw_fps_overlay(frame)

        if self.debug_view_scale <= 0.0 or self.debug_view_scale == 1.0:
            return frame

        width = max(int(frame.shape[1] * self.debug_view_scale), 1)
        height = max(int(frame.shape[0] * self.debug_view_scale), 1)
        return cv2.resize(
            frame,
            (width, height),
            interpolation=cv2.INTER_AREA,
        )

    def _update_fps(self):
        """Update smoothed FPS based on processed frames."""

        now_ns = self.get_clock().now().nanoseconds
        if self.last_fps_timestamp_ns > 0:
            delta_sec = (now_ns - self.last_fps_timestamp_ns) / 1e9
            if delta_sec > 0.0:
                instant_fps = 1.0 / delta_sec
                if self.smoothed_fps <= 0.0:
                    self.smoothed_fps = instant_fps
                else:
                    self.smoothed_fps = (
                        0.85 * self.smoothed_fps + 0.15 * instant_fps
                    )
        self.last_fps_timestamp_ns = now_ns

    def _draw_fps_overlay(self, frame: np.ndarray):
        """Draw FPS in the top-left corner of the debug frame."""

        text = f'{self.model_architecture.upper()} FPS: {self.smoothed_fps:.1f}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            text,
            font,
            font_scale,
            thickness,
        )

        x1 = 12
        y1 = 12
        x2 = x1 + text_width + 16
        y2 = y1 + text_height + baseline + 14

        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.70, frame, 0.30, 0.0, frame)
        cv2.putText(
            frame,
            text,
            (x1 + 8, y2 - baseline - 6),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _forward(
        self,
        input_tensor: np.ndarray,
    ) -> tuple[np.ndarray, ...] | list[np.ndarray] | None:
        """Run a single forward pass with the active backend."""

        if self.inference_backend == 'onnxruntime':
            if self.session is None:
                return None
            try:
                return self.session.run(
                    self.session_output_names,
                    {self.session_input_name: input_tensor},
                )
            except Exception as error:
                if not self.forward_error_logged:
                    self.get_logger().error(
                        f'ONNX Runtime forward failed: {error}'
                    )
                    self.forward_error_logged = True
                return None

        if self.net is None:
            return None

        self.net.setInput(input_tensor)
        try:
            return self.net.forward(self.output_names)
        except cv2.error as error:
            if not self.forward_error_logged:
                self.get_logger().error(
                    'Detector forward failed. The model is likely exported '
                    'for a different fixed input size or expects a different '
                    'input tensor layout.'
                )
                self.get_logger().error(str(error))
                self.forward_error_logged = True
            return None

    def _decode_yolo_prediction_row(
        self,
        row: np.ndarray,
        letterbox: LetterboxResult,
        frame_width: int,
        frame_height: int,
    ) -> tuple[int | None, float, list[int] | None]:
        """Decode one YOLO row into class, confidence, and xywh box."""

        if row.shape[0] < 4 + len(self.class_names):
            return None, 0.0, None

        cx, cy, width, height = row[:4]
        class_scores = row[4:]
        objectness = 1.0

        if row.shape[0] >= 5 + len(self.class_names):
            objectness = float(row[4])
            class_scores = row[5:]

        class_id = int(np.argmax(class_scores))
        class_score = float(class_scores[class_id])
        confidence = objectness * class_score

        if (
            confidence < self.confidence_threshold
            or class_score < self.score_threshold
        ):
            return None, confidence, None

        x1 = (cx - width / 2.0 - letterbox.pad_x) / letterbox.scale
        y1 = (cy - height / 2.0 - letterbox.pad_y) / letterbox.scale
        x2 = (cx + width / 2.0 - letterbox.pad_x) / letterbox.scale
        y2 = (cy + height / 2.0 - letterbox.pad_y) / letterbox.scale

        x1 = max(min(int(round(x1)), frame_width - 1), 0)
        y1 = max(min(int(round(y1)), frame_height - 1), 0)
        x2 = max(min(int(round(x2)), frame_width - 1), 0)
        y2 = max(min(int(round(y2)), frame_height - 1), 0)

        width_px = x2 - x1
        height_px = y2 - y1
        if width_px <= 1 or height_px <= 1:
            return None, confidence, None

        return class_id, confidence, [x1, y1, width_px, height_px]

    def _publish_detection_status(
        self,
        detections: list[Detection],
        frame_width: int,
        frame_height: int,
    ):
        """Publish summary topics for downstream logic."""

        labels = [detection.label for detection in detections]
        class_ids = [int(detection.class_id) for detection in detections]
        bounding_boxes: list[int] = []
        for detection in detections:
            bounding_boxes.extend(
                [
                    int(detection.class_id),
                    int(round(detection.confidence * 1000.0)),
                    int(detection.x1),
                    int(detection.y1),
                    int(detection.x2),
                    int(detection.y2),
                ]
            )

        has_target = self._has_target_detection(detections)
        target = self._best_target_detection(detections)
        target_bbox: list[int] = []
        if target is not None:
            target_bbox = [
                int(round(target.confidence * 1000.0)),
                int(round((target.x1 + target.x2) / 2.0)),
                int(round((target.y1 + target.y2) / 2.0)),
                int(target.x2 - target.x1),
                int(target.y2 - target.y1),
                int(frame_width),
                int(frame_height),
            ]
        now_ns = self.get_clock().now().nanoseconds
        if has_target:
            self.last_seen_target_ns = now_ns

        hold_period_ns = int(max(self.detection_hold_sec, 0.0) * 1e9)
        effective_has_target = has_target
        if (
            not effective_has_target
            and hold_period_ns > 0
            and self.last_seen_target_ns > 0
            and now_ns - self.last_seen_target_ns <= hold_period_ns
        ):
            effective_has_target = True

        self.target_detected_pub.publish(Bool(data=effective_has_target))
        self.detection_count_pub.publish(Int32(data=len(detections)))
        self.class_ids_pub.publish(Int32MultiArray(data=class_ids))
        self.bounding_boxes_pub.publish(
            Int32MultiArray(data=bounding_boxes)
        )
        self.target_bbox_pub.publish(Int32MultiArray(data=target_bbox))
        self.class_labels_pub.publish(String(data=','.join(labels)))

        self._log_target_summary(detections)

        if effective_has_target != self.last_detection_state:
            if effective_has_target:
                if target is not None:
                    self.get_logger().info(
                        'Target detected: '
                        f'label={target.label}, '
                        f'confidence={target.confidence:.2f}, '
                        f'bbox=({target.x1}, {target.y1})-'
                        f'({target.x2}, {target.y2}), '
                        f'classes_in_frame={labels or ["<none>"]}'
                    )
                else:
                    self.get_logger().info(
                        'Target detected. Classes in frame: '
                        f'{labels or ["<none>"]}'
                    )
            else:
                self.get_logger().info('Target lost')
            self.last_detection_state = effective_has_target

    def _has_target_detection(self, detections: list[Detection]) -> bool:
        """Return whether detections match the configured target set."""

        if not detections:
            return False

        if not self.target_labels_set:
            return True

        return any(
            detection.label.lower() in self.target_labels_set
            for detection in detections
        )

    def _best_target_detection(
        self,
        detections: list[Detection],
    ) -> Detection | None:
        """Return the highest-confidence detection among target labels."""

        if not self.target_labels_set:
            if not detections:
                return None
            return max(detections, key=lambda detection: detection.confidence)

        target_detections = [
            detection
            for detection in detections
            if detection.label.lower() in self.target_labels_set
        ]
        if not target_detections:
            return None
        return max(
            target_detections,
            key=lambda detection: detection.confidence,
        )

    def _log_target_summary(self, detections: list[Detection]):
        """Periodically log the strongest target detection clearly."""

        target = self._best_target_detection(detections)
        if target is None:
            return

        if self.target_summary_log_period_sec <= 0.0:
            return

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(self.target_summary_log_period_sec * 1e9)
        if now_ns - self.last_target_summary_log_ns < min_period_ns:
            return

        center_x = int(round((target.x1 + target.x2) / 2.0))
        center_y = int(round((target.y1 + target.y2) / 2.0))
        width = target.x2 - target.x1
        height = target.y2 - target.y1
        self.get_logger().info(
            'Tracking target: '
            f'label={target.label}, '
            f'confidence={target.confidence:.2f}, '
            f'center=({center_x}, {center_y}), '
            f'size={width}x{height}'
        )
        self.last_target_summary_log_ns = now_ns

    def _draw_detections(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> np.ndarray:
        """Draw detection boxes and labels on a frame."""

        for detection in detections:
            is_target = (
                not self.target_labels_set
                or detection.label.lower() in self.target_labels_set
            )
            color = (0, 0, 255) if is_target else (0, 180, 255)
            label_prefix = (
                f'{self.model_architecture.upper()} TARGET'
                if is_target
                else detection.label.upper()
            )
            label = f'{label_prefix} {detection.confidence:.2f}'
            box_thickness = 3 if is_target else 2

            cv2.rectangle(
                frame,
                (detection.x1, detection.y1),
                (detection.x2, detection.y2),
                color,
                box_thickness,
            )
            self._draw_label_badge(
                frame=frame,
                text=label,
                x=detection.x1,
                y=detection.y1,
                color=color,
            )

            if is_target:
                self._draw_target_accent(
                    frame=frame,
                    x1=detection.x1,
                    y1=detection.y1,
                    x2=detection.x2,
                    y2=detection.y2,
                    color=color,
                )

        return frame

    def _draw_label_badge(
        self,
        frame: np.ndarray,
        text: str,
        x: int,
        y: int,
        color: tuple[int, int, int],
    ):
        """Draw a filled label badge for a detection."""

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.58
        text_thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            text,
            font,
            font_scale,
            text_thickness,
        )

        badge_x1 = max(x, 0)
        badge_y2 = max(y - 6, text_height + baseline + 8)
        badge_y1 = max(badge_y2 - text_height - baseline - 10, 0)
        badge_x2 = min(
            badge_x1 + text_width + 14,
            frame.shape[1] - 1,
        )

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (badge_x1, badge_y1),
            (badge_x2, badge_y2),
            color,
            -1,
        )
        cv2.addWeighted(overlay, 0.88, frame, 0.12, 0.0, frame)
        cv2.putText(
            frame,
            text,
            (badge_x1 + 7, badge_y2 - baseline - 4),
            font,
            font_scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )

    def _draw_target_accent(
        self,
        frame: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color: tuple[int, int, int],
    ):
        """Draw corner accents and center point for the target class."""

        width = max(x2 - x1, 1)
        height = max(y2 - y1, 1)
        corner = max(min(width, height) // 5, 12)
        accent_thickness = 3

        corners = (
            ((x1, y1), (x1 + corner, y1), (x1, y1 + corner)),
            ((x2, y1), (x2 - corner, y1), (x2, y1 + corner)),
            ((x1, y2), (x1 + corner, y2), (x1, y2 - corner)),
            ((x2, y2), (x2 - corner, y2), (x2, y2 - corner)),
        )
        for anchor, horizontal, vertical in corners:
            cv2.line(frame, anchor, horizontal, color, accent_thickness)
            cv2.line(frame, anchor, vertical, color, accent_thickness)

        center_x = int(round((x1 + x2) / 2.0))
        center_y = int(round((y1 + y2) / 2.0))
        cv2.circle(frame, (center_x, center_y), 4, color, -1)

    def _label_for_class_id(self, class_id: int) -> str:
        """Return a label for a class index."""

        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f'class_{class_id}'

    def _should_process_frame(self) -> bool:
        """Throttle inference to keep CPU use reasonable."""

        if self.processing_max_rate_hz <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(1e9 / self.processing_max_rate_hz)
        if now_ns - self.last_processed_frame_ns < min_period_ns:
            return False

        self.last_processed_frame_ns = now_ns
        return True

    def _should_publish_debug_image(self) -> bool:
        """Throttle debug image publication."""

        if not self.publish_debug_image:
            return False

        if self.debug_image_max_rate_hz <= 0.0:
            return True

        now_ns = self.get_clock().now().nanoseconds
        min_period_ns = int(1e9 / self.debug_image_max_rate_hz)
        if now_ns - self.last_debug_image_pub_ns < min_period_ns:
            return False

        self.last_debug_image_pub_ns = now_ns
        return True

    def _bgr_to_image_message(
        self,
        frame: np.ndarray,
        stamp,
        frame_id: str,
    ) -> Image:
        """Convert a BGR OpenCV frame into a ROS Image message."""

        contiguous_frame = np.ascontiguousarray(frame)
        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id
        image_msg.height = int(contiguous_frame.shape[0])
        image_msg.width = int(contiguous_frame.shape[1])
        image_msg.encoding = 'bgr8'
        image_msg.is_bigendian = False
        image_msg.step = int(contiguous_frame.strides[0])
        image_msg.data = contiguous_frame.tobytes()
        return image_msg

    def _image_to_bgr(self, msg: Image) -> np.ndarray | None:
        """Convert a ROS Image to an OpenCV BGR frame."""

        channel_map = {
            'mono8': 1,
            '8UC1': 1,
            'rgb8': 3,
            'bgr8': 3,
            '8UC3': 3,
            'rgba8': 4,
            'bgra8': 4,
        }
        channels = channel_map.get(msg.encoding)
        if channels is None:
            self.get_logger().error(
                'Unsupported image encoding for detection: '
                f'{msg.encoding}'
            )
            return None

        frame_buffer = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        min_row_stride = int(msg.width) * channels
        if row_stride < min_row_stride:
            self.get_logger().error(
                f'Invalid image step {row_stride} for encoding {msg.encoding}'
            )
            return None

        expected_size = row_stride * int(msg.height)
        if frame_buffer.size < expected_size:
            self.get_logger().error(
                f'Image buffer too small: got {frame_buffer.size} bytes, '
                f'expected at least {expected_size}'
            )
            return None

        frame = frame_buffer[:expected_size].reshape(
            (int(msg.height), row_stride)
        )
        frame = frame[:, :min_row_stride].reshape(
            int(msg.height),
            int(msg.width),
            channels,
        ) if channels > 1 else frame[:, :min_row_stride].reshape(
            int(msg.height),
            int(msg.width),
        )

        if msg.encoding in {'bgr8', '8UC3'}:
            return np.ascontiguousarray(frame)
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding in {'rgba8', 'bgra8'}:
            conversion = (
                cv2.COLOR_RGBA2BGR
                if msg.encoding == 'rgba8'
                else cv2.COLOR_BGRA2BGR
            )
            return cv2.cvtColor(frame, conversion)
        if msg.encoding in {'mono8', '8UC1'}:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return None


def main(args=None):
    """Run the detector node."""

    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
