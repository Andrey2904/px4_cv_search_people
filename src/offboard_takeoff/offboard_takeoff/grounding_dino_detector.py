"""ROS 2 Grounding DINO detector node for prompt-based person detection."""

from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path


def _extend_sys_path_with_local_venv():
    """Add the workspace model venv to sys.path when available."""

    current_path = Path(__file__).resolve()
    for base in [current_path.parent, *current_path.parents]:
        candidate = base / 'models' / '.venv' / 'lib'
        if not candidate.is_dir():
            continue

        python_dirs = sorted(candidate.glob('python*/site-packages'))
        for python_dir in python_dirs:
            path_string = str(python_dir)
            if path_string not in sys.path:
                sys.path.insert(0, path_string)
        if python_dirs:
            return


_extend_sys_path_with_local_venv()

import cv2
import numpy as np
from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

try:
    import torch
except ImportError:
    torch = None

try:
    from transformers import AutoModelForZeroShotObjectDetection
    from transformers import AutoProcessor
except ImportError:
    AutoModelForZeroShotObjectDetection = None
    AutoProcessor = None


@dataclass(frozen=True)
class Detection:
    """Single object detection in image coordinates."""

    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int


class GroundingDinoDetector(Node):
    """Prompt-conditioned detector based on Grounding DINO."""

    def __init__(self):
        super().__init__('grounding_dino_detector')

        self._declare_parameters()
        self._load_parameters()
        self._initialize_model()

        self.target_labels_set = {
            label.strip().lower()
            for label in self.target_labels
            if label.strip()
        }

        self.last_processed_frame_ns = 0
        self.last_debug_image_pub_ns = 0
        self.last_detection_state = False
        self.last_seen_target_ns = 0
        self.last_target_summary_log_ns = 0
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
            'Grounding DINO detector started: '
            f'image_topic={self.image_topic}, '
            f'model_id={self.model_id}, '
            f'prompt={self.prompt_text}, '
            f'device={self.device}, '
            f'processing_max_rate_hz={self.processing_max_rate_hz:.2f}'
        )

    def _declare_parameters(self):
        """Declare ROS parameters."""

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'model_id',
            'IDEA-Research/grounding-dino-tiny',
        )
        self.declare_parameter('cache_dir', '')
        self.declare_parameter('prompt_text', 'person')
        self.declare_parameter('target_labels', ['person'])
        self.declare_parameter('box_threshold', 0.35)
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('processing_max_rate_hz', 2.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_max_rate_hz', 2.0)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('debug_view_scale', 0.75)
        self.declare_parameter('show_fps_overlay', True)
        self.declare_parameter('detection_hold_sec', 0.75)
        self.declare_parameter('prefer_cuda', True)
        self.declare_parameter('use_half', True)
        self.declare_parameter('local_files_only', False)
        self.declare_parameter('target_summary_log_period_sec', 1.5)

    def _load_parameters(self):
        """Load ROS parameters into node fields."""

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.model_id = str(self.get_parameter('model_id').value).strip()
        self.cache_dir = str(self.get_parameter('cache_dir').value).strip()
        self.prompt_text = self._normalize_prompt(
            str(self.get_parameter('prompt_text').value)
        )
        self.target_labels = list(self.get_parameter('target_labels').value)
        self.box_threshold = float(
            self.get_parameter('box_threshold').value
        )
        self.text_threshold = float(
            self.get_parameter('text_threshold').value
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
        self.prefer_cuda = bool(self.get_parameter('prefer_cuda').value)
        self.use_half = bool(self.get_parameter('use_half').value)
        self.local_files_only = bool(
            self.get_parameter('local_files_only').value
        )
        self.target_summary_log_period_sec = float(
            self.get_parameter('target_summary_log_period_sec').value
        )

    def _normalize_prompt(self, prompt_text: str) -> str:
        """Normalize Grounding DINO prompt syntax."""

        normalized = prompt_text.strip()
        if not normalized:
            normalized = 'person'
        if not normalized.endswith('.'):
            normalized = f'{normalized}.'
        return normalized

    def _initialize_model(self):
        """Load processor and model."""

        if torch is None:
            raise ImportError(
                'torch is required for Grounding DINO detection.'
            )
        if (
            AutoProcessor is None
            or AutoModelForZeroShotObjectDetection is None
        ):
            raise ImportError(
                'transformers is required for Grounding DINO detection.'
            )

        self.device = 'cpu'
        if self.prefer_cuda and torch.cuda.is_available():
            self.device = 'cuda'
        elif self.prefer_cuda:
            self.get_logger().warning(
                'prefer_cuda=true, but CUDA is unavailable. '
                'Grounding DINO will run on CPU.'
            )

        load_kwargs = {}
        if self.cache_dir:
            load_kwargs['cache_dir'] = self.cache_dir
        if self.local_files_only:
            load_kwargs['local_files_only'] = True

        self.processor = AutoProcessor.from_pretrained(
            self.model_id,
            **load_kwargs,
        )
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.model_id,
            **load_kwargs,
        )
        self.model.to(self.device)
        self.model.eval()

        self.model_dtype = torch.float32
        if self.device == 'cuda' and self.use_half:
            self.get_logger().warning(
                'use_half=true requested, but Grounding DINO is kept in '
                'float32 for stability to avoid dtype mismatches on GPU.'
            )

    def _image_callback(self, msg: Image):
        """Run prompt-based detection on the incoming image."""

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

        debug_frame = self._draw_detections(frame.copy(), detections)
        prepared_debug_frame = self._prepare_debug_frame(debug_frame)

        if self.show_debug_window:
            cv2.imshow('Grounding DINO Debug View', prepared_debug_frame)
            cv2.waitKey(1)

        if self._should_publish_debug_image():
            debug_msg = self._bgr_to_image_message(
                frame=prepared_debug_frame,
                stamp=msg.header.stamp,
                frame_id=msg.header.frame_id,
            )
            self.debug_image_pub.publish(debug_msg)

    def _run_inference(self, frame: np.ndarray) -> list[Detection]:
        """Run Grounding DINO inference."""

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = PilImage.fromarray(rgb_frame)
        inputs = self.processor(
            images=image,
            text=self.prompt_text,
            return_tensors='pt',
        )
        for key, value in inputs.items():
            if hasattr(value, 'to'):
                if key == 'pixel_values':
                    inputs[key] = value.to(
                        self.device,
                        dtype=self.model_dtype,
                    )
                else:
                    inputs[key] = value.to(self.device)

        with torch.inference_mode():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            target_sizes=[image.size[::-1]],
        )
        return self._detections_from_result(results[0], frame.shape[1], frame.shape[0])

    def _detections_from_result(
        self,
        result,
        frame_width: int,
        frame_height: int,
    ) -> list[Detection]:
        """Convert HF Grounding DINO outputs into Detection objects."""

        boxes = result.get('boxes', [])
        scores = result.get('scores', [])
        labels = result.get('labels', [])

        detections: list[Detection] = []
        for box, score, label in zip(boxes, scores, labels):
            confidence = float(score.item() if hasattr(score, 'item') else score)
            label_text = str(label).strip().lower()
            x1, y1, x2, y2 = [
                int(round(float(value.item() if hasattr(value, 'item') else value)))
                for value in box
            ]
            x1 = max(min(x1, frame_width - 1), 0)
            y1 = max(min(y1, frame_height - 1), 0)
            x2 = max(min(x2, frame_width - 1), 0)
            y2 = max(min(y2, frame_height - 1), 0)
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(
                Detection(
                    label=label_text,
                    confidence=confidence,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )

        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def _publish_detection_status(
        self,
        detections: list[Detection],
        frame_width: int,
        frame_height: int,
    ):
        """Publish summary topics compatible with the mission node."""

        labels = [detection.label for detection in detections]
        class_ids = list(range(len(detections)))
        bounding_boxes: list[int] = []
        for index, detection in enumerate(detections):
            bounding_boxes.extend(
                [
                    int(index),
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
            if effective_has_target and target is not None:
                self.get_logger().info(
                    'Grounding DINO target detected: '
                    f'label={target.label}, '
                    f'confidence={target.confidence:.2f}, '
                    f'bbox=({target.x1}, {target.y1})-({target.x2}, {target.y2})'
                )
            elif not effective_has_target:
                self.get_logger().info('Grounding DINO target lost')
            self.last_detection_state = effective_has_target

    def _has_target_detection(self, detections: list[Detection]) -> bool:
        """Return whether detections match configured target labels."""

        if not detections:
            return False
        return any(
            any(target in detection.label for target in self.target_labels_set)
            for detection in detections
        )

    def _best_target_detection(
        self,
        detections: list[Detection],
    ) -> Detection | None:
        """Return the best detection among target labels."""

        matches = [
            detection
            for detection in detections
            if any(target in detection.label for target in self.target_labels_set)
        ]
        if not matches:
            return None
        return max(matches, key=lambda detection: detection.confidence)

    def _log_target_summary(self, detections: list[Detection]):
        """Periodically log the strongest detection."""

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
        self.get_logger().info(
            'Grounding DINO tracking target: '
            f'label={target.label}, '
            f'confidence={target.confidence:.2f}, '
            f'center=({center_x}, {center_y})'
        )
        self.last_target_summary_log_ns = now_ns

    def _draw_detections(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> np.ndarray:
        """Draw detections on a frame."""

        for detection in detections:
            is_target = any(
                target in detection.label for target in self.target_labels_set
            )
            color = (0, 0, 255) if is_target else (0, 180, 255)
            label = f'{detection.label} {detection.confidence:.2f}'
            thickness = 3 if is_target else 2

            cv2.rectangle(
                frame,
                (detection.x1, detection.y1),
                (detection.x2, detection.y2),
                color,
                thickness,
            )
            cv2.putText(
                frame,
                label,
                (detection.x1, max(detection.y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        return frame

    def _prepare_debug_frame(self, frame: np.ndarray) -> np.ndarray:
        """Optionally downscale and annotate the debug frame."""

        if self.show_fps_overlay:
            text = f'GDINO FPS: {self.smoothed_fps:.1f}'
            cv2.putText(
                frame,
                text,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.debug_view_scale <= 0.0 or self.debug_view_scale == 1.0:
            return frame

        width = max(int(frame.shape[1] * self.debug_view_scale), 1)
        height = max(int(frame.shape[0] * self.debug_view_scale), 1)
        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

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

    def _should_process_frame(self) -> bool:
        """Throttle inference to keep compute use reasonable."""

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
        """Convert a BGR frame into a ROS Image message."""

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
                'Unsupported image encoding for Grounding DINO: '
                f'{msg.encoding}'
            )
            return None

        frame_buffer = np.frombuffer(msg.data, dtype=np.uint8)
        row_stride = int(msg.step)
        min_row_stride = int(msg.width) * channels
        expected_size = row_stride * int(msg.height)
        if row_stride < min_row_stride or frame_buffer.size < expected_size:
            self.get_logger().error('Invalid image buffer for Grounding DINO')
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
    """Run the Grounding DINO detector node."""

    rclpy.init(args=args)
    node = GroundingDinoDetector()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
