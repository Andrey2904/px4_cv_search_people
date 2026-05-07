# Детекция людей YOLO

## Назначение подсистемы

Подсистема детекции людей выделяет пострадавшего на изображении с бортовой
камеры квадрокоптера и передает результат в миссионную логику через ROS 2
topics семейства `yolo/*`.

В текущем рабочем сценарии используется YOLO12s: квадрокоптер PX4 в симуляции
Gazebo GZ летит по поисковому маршруту, получает изображения с камеры модели
`gz_x500_mono_cam`, детектирует людей, публикует лучший bounding box и затем
использует этот bbox для подтверждения цели и визуального наведения.

## Где реализовано

Основные файлы:

- `src/offboard_takeoff/offboard_takeoff/yolo_detector.py` - основной ROS 2
  detector node;
- `src/offboard_takeoff/config/yolo_detector.yaml` - YAML-конфигурация
  YOLO-детектора;
- `src/offboard_takeoff/launch/person_detection.launch.py` - предпочтительная
  точка запуска camera pipeline и YOLO-детектора;
- `training/yolo/README.md`, `training/yolo/data_person.yaml` - описание
  датасета и процесса fine-tuning;
- `tools/audit_yolo_dataset.py`, `tools/train_yolo.py`,
  `tools/export_yolo_onnx.py`, `tools/predict_video_yolo.py` - утилиты для
  проверки датасета, обучения, экспорта и офлайн-проверки модели.

## Роль YOLO12s

YOLO12s является основной нейросетевой моделью для детекции людей в
симуляционной поисково-спасательной миссии. В корневом `README.md` проект
описан как `PX4 SITL Person Search with YOLO12s for gz_x500_mono_cam`.

Система:

1. получает изображения с камеры квадрокоптера;
2. выполняет inference модели YOLO;
3. выбирает лучший bbox человека;
4. публикует его в `yolo/target_bbox`;
5. передает компактные данные bbox в миссионный конечный автомат.

В launch-файлах и конфигурации в качестве основного checkpoint указан путь:

```text
/home/dron/.gz/models/yolo12n_people_package/runs/yolo12s_people_e30_b62/weights/best.pt
```

По коду этот файл используется как Ultralytics `.pt` checkpoint. В
`tools/train_yolo.py` базовая модель обучения по умолчанию также задана как
`yolo12s.pt`. Численные метрики качества, например mAP, precision или recall,
в репозитории не зафиксированы, поэтому их нельзя указывать в дипломе без
отдельного эксперимента и протокола измерений.

## Входное изображение

Основной вход YOLO-детектора - ROS 2 сообщение:

```text
sensor_msgs/msg/Image
```

Параметр входного topic:

```text
image_topic
```

Значение по умолчанию в `yolo_detector.py` и
`src/offboard_takeoff/config/yolo_detector.yaml`:

```text
/camera/image_raw
```

При запуске через `person_detection.launch.py` camera topic может быть найден
автоматически среди Gazebo topics и передан в параметр `image_topic`.
Ожидаемый вид Gazebo image topic:

```text
/world/<world>/model/<model>/link/<link>/sensor/<sensor>/image
```

В `yolo_detector.py` поддерживаются следующие encodings:

- `mono8`;
- `8UC1`;
- `rgb8`;
- `bgr8`;
- `8UC3`;
- `rgba8`;
- `bgra8`.

Внутри detector node изображения приводятся к BGR-кадру OpenCV. Для `rgb8`
выполняется преобразование RGB -> BGR, для `rgba8`/`bgra8` отбрасывается alpha
channel, для grayscale выполняется преобразование в BGR.

## Camera pipeline

По README запуск detection pipeline делает три вещи:

1. находит реальные camera topics в Gazebo;
2. запускает `ros_gz_bridge` для изображения, camera info и `/clock`;
3. запускает `camera_viewer` и, при `start_yolo:=true`, `yolo_detector`.

Если `enable_bridge:=true`, создается bridge node:

```text
package='ros_gz_bridge'
executable='parameter_bridge'
name='gz_camera_bridge'
```

Bridge передает:

```text
<gazebo_image_topic>@sensor_msgs/msg/Image[gz.msgs.Image
<gazebo_camera_info_topic>@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

`/clock` добавляется только при включенном `bridge_clock`.

`camera_viewer` нужен как relay/viewer: он принимает image и camera info,
может показывать окно OpenCV и публикует relay topics:

```text
/camera/image_raw
/camera/camera_info
```

## Backend-и inference

`yolo_detector.py` поддерживает несколько способов запуска YOLO-модели.

### Ultralytics `.pt`

Если путь к модели заканчивается на `.pt`, используется backend
`ultralytics`. Он требует установленный пакет `ultralytics`.

При `prefer_gpu=True` и доступной CUDA detector выбирает:

```text
device = cuda:0
half = True
```

Иначе inference выполняется на CPU.

Для `.pt` checkpoint в режиме `auto` вызывается:

```text
UltralyticsYOLO(model_path)
model.predict(...)
```

### ONNX Runtime

Для `.onnx` модели в режиме `auto` сначала пробуется ONNX Runtime. Если
доступен `CUDAExecutionProvider`, он добавляется первым, после чего всегда
добавляется `CPUExecutionProvider`.

Если ONNX Runtime отсутствует или не инициализируется в режиме `auto`, код
переходит к OpenCV DNN. Если явно задан `inference_backend:=onnxruntime`,
ошибка инициализации считается фатальной.

### OpenCV DNN

OpenCV DNN используется как fallback для ONNX-моделей или как явный backend:

```text
inference_backend:=opencv
```

Инициализация выполняется через:

```text
cv2.dnn.readNetFromONNX(...)
cv2.dnn.DNN_BACKEND_OPENCV
cv2.dnn.DNN_TARGET_CPU
```

В текущем коде OpenCV DNN работает на CPU.

## Основные параметры

Параметры по умолчанию в `yolo_detector.py`:

| Параметр | Значение в коде | Смысл |
|---|---:|---|
| `image_topic` | `/camera/image_raw` | входной ROS image topic |
| `model_path` | пусто | путь к `.pt` или `.onnx`; если пусто, запускается auto-discovery |
| `model_architecture` | `yolo` | режим декодирования outputs |
| `inference_backend` | `auto` | выбор backend-а |
| `prefer_gpu` | `True` | использовать CUDA, если доступна |
| `target_labels` | `['person']` | какие классы считать целью |
| `input_width` | `640` | ширина входа модели |
| `input_height` | `640` | высота входа модели |
| `confidence_threshold` | `0.35` | минимальная итоговая confidence |
| `score_threshold` | `0.25` | минимальный class score для YOLO outputs |
| `nms_threshold` | `0.45` | IoU-порог NMS |
| `processing_max_rate_hz` | `3.0` | ограничение частоты обработки кадров |
| `publish_debug_image` | `False` | публиковать `yolo/debug_image` |
| `debug_image_max_rate_hz` | `2.0` | ограничение частоты debug image |
| `show_debug_window` | `True` | показывать OpenCV debug window |
| `detection_hold_sec` | `0.75` | удержание boolean-флага target detected |

В `src/offboard_takeoff/config/yolo_detector.yaml` для рабочего YOLO-сценария
заданы:

```yaml
model_architecture: yolo
inference_backend: ultralytics
input_width: 960
input_height: 960
confidence_threshold: 0.35
score_threshold: 0.25
nms_threshold: 0.45
processing_max_rate_hz: 5.0
publish_debug_image: true
debug_image_max_rate_hz: 2.0
detection_hold_sec: 0.75
```

## Auto-discovery модели

Если `model_path` пустой, `yolo_detector.py` пытается найти модель
автоматически.

Сначала проверяются предпочтительные внешние пути в:

```text
/home/dron/.gz/models/yolo12n_people_package/runs/...
```

В числе первых кандидатов:

```text
yolo12s_people_e30_b62/weights/best.pt
yolo12s_people_e30_b62/weights/last.pt
```

Если preferred paths не найдены, detector ищет новые `.onnx`, `best.pt` и
`last.pt` в локальных `models/yolo_runs` и в `/home/dron/.gz/models`.
Выбирается самый свежий файл по времени изменения.

Если модель не найдена, node бросает ошибку и просит явно передать путь к
`.onnx` или `.pt`.

## Алгоритм обработки кадра

Callback `_image_callback()` выполняет:

1. проверку ограничения частоты `_should_process_frame()`;
2. конвертацию ROS image в BGR-кадр OpenCV;
3. inference через выбранный backend;
4. публикацию summary topics;
5. обновление FPS;
6. при необходимости показ или публикацию debug image.

Для YOLO ONNX/OpenCV path используется letterbox:

1. исходный кадр масштабируется с сохранением пропорций;
2. пустые области заполняются padding;
3. формируется blob `cv2.dnn.blobFromImage`;
4. выполняется forward;
5. outputs приводятся к единому виду;
6. декодируются `class_id`, confidence и bbox;
7. bbox проецируется обратно в координаты исходного кадра;
8. выполняется NMS;
9. detections сортируются по confidence по убыванию.

Для Ultralytics `.pt` большую часть decode/NMS выполняет библиотека
`ultralytics`, а node преобразует результат к внутреннему dataclass
`Detection`.

## Публикуемые topics

YOLO-детектор публикует:

| Topic | Тип | Содержимое |
|---|---|---|
| `yolo/debug_image` | `sensor_msgs/msg/Image` | кадр с нарисованными detections, если включена публикация debug image |
| `yolo/target_detected` | `std_msgs/msg/Bool` | есть ли целевая detection с учетом `detection_hold_sec` |
| `yolo/detection_count` | `std_msgs/msg/Int32` | число detections в текущем обработанном кадре |
| `yolo/class_ids` | `std_msgs/msg/Int32MultiArray` | class IDs всех detections |
| `yolo/bounding_boxes` | `std_msgs/msg/Int32MultiArray` | плоский массив bbox всех detections |
| `yolo/target_bbox` | `std_msgs/msg/Int32MultiArray` | компактный bbox лучшей целевой detection |
| `yolo/class_labels` | `std_msgs/msg/String` | labels detections через запятую |

### Формат `yolo/bounding_boxes`

Для каждого detection публикуется 6 чисел:

```text
class_id, confidence_milli, x1, y1, x2, y2
```

Где:

- `confidence_milli = round(confidence * 1000)`;
- `x1, y1, x2, y2` - координаты bbox в пикселях исходного изображения.

### Формат `yolo/target_bbox`

Это главный интерфейс с миссионной логикой. Если target найден, публикуется
массив из 7 чисел:

```text
confidence_milli, center_x, center_y, width, height, frame_width, frame_height
```

Где:

- `confidence_milli = round(confidence * 1000)`;
- `center_x`, `center_y` - центр bbox в пикселях;
- `width`, `height` - размер bbox в пикселях;
- `frame_width`, `frame_height` - размер исходного кадра.

Если target не найден, публикуется пустой массив.

Цель выбирается как detection с максимальной confidence среди labels,
совпадающих с `target_labels`. Для рабочего сценария `target_labels` содержит
`person`.

### `detection_hold_sec`

Boolean topic `yolo/target_detected` удерживается короткое время после
последнего обнаружения:

- если target был найден, сохраняется timestamp последнего обнаружения;
- если target временно пропал, но время с последнего обнаружения меньше
  `detection_hold_sec`, `yolo/target_detected` продолжает публиковаться как
  `true`;
- `target_bbox` при этом формируется только если target реально присутствует
  в текущем кадре.

Это снижает чувствительность boolean-флага к кратковременным пропускам, но не
подменяет геометрию bbox.

## Debug image и debug viewer

В `yolo_detector.py` debug frame формируется после inference: на копию кадра
наносятся bbox, labels, confidence, размер bbox и FPS overlay, если
`show_fps_overlay=True`.

Если `show_debug_window=True`, кадр показывается в OpenCV-окне detector debug
view. Если `publish_debug_image=True`, подготовленный debug frame публикуется
в topic:

```text
yolo/debug_image
```

Публикация ограничивается параметром:

```text
debug_image_max_rate_hz
```

В YAML-конфигурации debug publication включена:

```text
publish_debug_image: true
debug_image_max_rate_hz: 2.0
```

## Обучение YOLO

Папка `training/yolo` содержит training assets для fine-tuning single-class
YOLO detector на локальном drone dataset.

Датасет описан в `training/yolo/data_person.yaml`. Файл задает локальный путь к
датасету, папки train/val и единственный класс:

```yaml
train: train/images
val: val/images
names:
  0: person
```

Важно: здесь приведен только путь, указанный в YAML. Само наличие такого
локального датасета и его состав документ не проверяет.

`training/yolo/README.md` описывает workflow:

1. проверить датасет с `tools/audit_yolo_dataset.py`;
2. обучить модель через `scripts/train_yolo_gpu.sh`;
3. экспортировать best checkpoint через `tools/export_yolo_onnx.py`.

### Проверка датасета

`tools/audit_yolo_dataset.py` проверяет:

- наличие `train/images`, `train/labels`, `val/images`, `val/labels`;
- наличие изображений;
- наличие label-файлов;
- корректность строк YOLO-label;
- class id;
- координаты `x_center`, `y_center`, `width`, `height` в диапазоне `[0, 1]`;
- количество пустых label-файлов;
- совпадение basename изображений и labels.

### Обучение

`tools/train_yolo.py` использует Ultralytics `YOLO`. Основные аргументы:

- `--model`, по умолчанию `yolo12s.pt`;
- `--data`, путь к dataset yaml;
- `--epochs`;
- `--imgsz`;
- `--batch`;
- `--device`;
- `--project`;
- `--name`.

Скрипт `scripts/train_yolo_gpu.sh` запускает обучение с GPU-настройками и
предварительно вызывает аудит датасета.

### Экспорт ONNX

`tools/export_yolo_onnx.py` экспортирует trained `.pt` checkpoint в ONNX через
Ultralytics `model.export(format='onnx', ...)`.

Параметры экспорта:

- `--weights`;
- `--imgsz`;
- `--opset`;
- `--dynamic`;
- `--simplify`;
- `--half`.

### Офлайн-проверка на видео

`tools/predict_video_yolo.py` запускает Ultralytics prediction на видеофайле и
сохраняет результат.

## Связь с миссионной логикой

Миссионный узел `OffboardTakeoff` подписывается на:

```text
yolo/target_bbox
```

В callback `person_target_callback()` массив преобразуется в dataclass
`PersonTarget`:

- `confidence`;
- `center_x`;
- `center_y`;
- `width`;
- `height`;
- `image_width`;
- `image_height`.

Затем вычисляется нормированная горизонтальная ошибка:

```text
center_x_error = (center_x - image_width / 2) / (image_width / 2)
```

Эта ошибка используется для yaw alignment в состоянии `FOLLOW_PERSON`.
Высота bbox используется как косвенный индикатор приближения: при движении
вперед миссия ожидает, что bbox человека станет больше относительно исходного
размера.

## Типовой запуск

Команда из README:

```bash
ros2 launch offboard_takeoff person_detection.launch.py \
  start_yolo:=true \
  detector_architecture:=yolo \
  yolo_model_path:=/home/dron/.gz/models/yolo12n_people_package/runs/yolo12s_people_e30_b62/weights/best.pt \
  yolo_inference_backend:=ultralytics \
  yolo_input_width:=960 \
  yolo_input_height:=960
```

Что происходит:

1. launch находит camera topics Gazebo;
2. при `enable_bridge=true` запускается `ros_gz_bridge`;
3. при `start_viewer=true` запускается `camera_viewer`;
4. при `start_yolo=true` запускается `yolo_detector`;
5. YOLO-детектор читает кадры, выполняет inference и публикует `yolo/*`.

Если bridge уже запущен, README предлагает:

```bash
ros2 launch offboard_takeoff person_detection.launch.py enable_bridge:=false
```

Если auto-discovery не подходит, README показывает ручное указание topics:

```bash
ros2 launch offboard_takeoff person_detection.launch.py \
  image_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image \
  camera_info_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info
```

## Ограничения и честные замечания

1. В репозитории нет численных результатов качества модели. Нельзя честно
   утверждать конкретные значения accuracy, mAP, precision, recall или FPS без
   отдельного запуска и протокола измерений.

2. Основной checkpoint указан как локальный абсолютный путь в
   `/home/dron/.gz/models/...`. Для переносимости репозитория потребуется
   отдельно документировать получение модели или хранение весов.

3. В README говорится о fine-tuned `YOLO12s`, но путь содержит директорию
   `yolo12n_people_package`. По README и `tools/train_yolo.py` основная модель
   называется `YOLO12s`; имя директории само по себе не доказывает архитектуру
   checkpoint.

4. В YAML `processing_max_rate_hz=5.0`, в `yolo_detector.py` default `3.0`, а
   в launch argument `yolo_processing_max_rate_hz=0.0`. Поэтому фактическая
   частота inference зависит от способа запуска.

5. Публикация `yolo/debug_image` зависит от `publish_debug_image`. Publisher
   создается всегда, но кадры публикуются только при включенном параметре.

6. Детектор выдает 2D bbox в координатах изображения. В перечисленных файлах
   нет вычисления реальной 3D-позиции человека или дальности до него на основе
   глубины. Для миссионной логики передается только центр, размер bbox и размер
   кадра.

7. OpenCV DNN backend в текущем коде настроен на CPU. GPU-ускорение для этого
   backend-а не используется.

## Формулировка для диплома

В рамках работы разработана и интегрирована подсистема нейросетевой детекции
людей на изображениях с бортовой камеры квадрокоптера. Основным detector
backend является дообученная модель YOLO12s, запускаемая как ROS 2 node и
принимающая поток `sensor_msgs/Image` из симуляционной среды Gazebo GZ.
Передача изображений из Gazebo в ROS 2 обеспечивается связкой
`ros_gz_bridge` и camera relay/viewer, запускаемой через launch-файлы проекта.

Детектор поддерживает запуск Ultralytics checkpoint `.pt`, экспортированных
ONNX-моделей через ONNX Runtime и fallback через OpenCV DNN. Результаты
публикуются в ROS 2 topics `yolo/*`, включая ключевой topic
`yolo/target_bbox`, содержащий confidence, центр и размер bounding box
целевого объекта в пикселях.

Такая организация отделяет алгоритм восприятия от миссионной логики:
автономный полетный сценарий получает не изображение целиком, а компактное
описание обнаруженной цели. Это упрощает интеграцию YOLO-детектора с конечным
автоматом миссии, визуальным наведением и процедурой подтверждения цели в
поисково-спасательном сценарии.

## Источники внутри репозитория

Документ составлен по следующим файлам проекта:

- `README.md`
- `training/yolo/README.md`
- `training/yolo/data_person.yaml`
- `training/yolo/requirements.txt`
- `src/offboard_takeoff/config/yolo_detector.yaml`
- `src/offboard_takeoff/offboard_takeoff/yolo_detector.py`
- `src/offboard_takeoff/launch/person_detection.launch.py`
- `tools/audit_yolo_dataset.py`
- `tools/train_yolo.py`
- `tools/export_yolo_onnx.py`
- `tools/predict_video_yolo.py`
