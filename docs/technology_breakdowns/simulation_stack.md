# Симуляционный стек проекта

## Назначение

Симуляционный стек обеспечивает среду, в которой можно проверить работу
поисково-спасательного квадрокоптера без реального БПЛА. В проекте
используются:

- PX4 SITL как программная имитация автопилота;
- Gazebo GZ как физическая и визуальная среда;
- `MicroXRCEAgent` как канал связи PX4 с ROS 2;
- ROS 2-узлы пакета `offboard_takeoff` для миссии, детекции и визуализации;
- `ros_gz_bridge` для передачи изображения камеры и `/clock` из Gazebo в
  ROS 2.

Идея разделения такая: PX4 и Gazebo моделируют летательный аппарат и мир, а
ROS 2 реализует логику поиска, детекции, маршрутов и визуализации.

## Где реализовано

Основные файлы:

| Файл | Роль |
| --- | --- |
| `README.md` | Основной порядок сборки и запуска PX4 SITL, `MicroXRCEAgent` и vision launch. |
| `src/offboard_takeoff/package.xml` | ROS 2-зависимости пакета. |
| `src/offboard_takeoff/setup.py` | Установка launch/config-файлов и объявление console script-ов. |
| `src/offboard_takeoff/launch/person_detection.launch.py` | Пользовательская точка запуска камеры, `camera_viewer` и YOLO-детектора. |
| `src/offboard_takeoff/launch/scene_map.launch.py` | Запуск публикации карты сцены и, опционально, RViz/top-down viewer. |
| `src/offboard_takeoff/launch/top_down_map.launch.py` | Отдельный запуск OpenCV-карты сверху. |
| `src/offboard_takeoff/offboard_takeoff/camera_viewer.py` | ROS 2-релей изображения и `CameraInfo`, overlay последней цели. |
| `src/offboard_takeoff/offboard_takeoff/node.py` | Миссионный узел PX4 Offboard: `/fmu/out/*`, `/fmu/in/*`, A*, поиск, возврат, телепорт модели. |
| `src/offboard_takeoff/config/yolo_detector.yaml` | Конфигурация YOLO-детектора. |
| `src/px4_msgs/README.md` | Описание пакета `px4_msgs` как ROS 2-интерфейсов PX4. |

## Запуск по README

В корневом `README.md` запуск разложен на несколько терминалов. Текущий
репозиторий не содержит одного общего launch-файла, который поднимает весь
стек от PX4 до миссии.

### Терминал 1: PX4 SITL + Gazebo GZ

```bash
cd ~/PX4-Autopilot
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500_mono_cam
```

Эта команда запускает PX4 SITL и Gazebo GZ-модель квадрокоптера X500 с
монокулярной камерой.

### Терминал 2: Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Этот процесс нужен для обмена между PX4 и ROS 2. Через него ROS 2-узлы
получают PX4-состояние и публикуют управляющие сообщения.

### Терминал 3: camera bridge и YOLO

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

Этот запуск:

1. находит camera topics Gazebo;
2. запускает `ros_gz_bridge` для изображения, `CameraInfo` и `/clock`;
3. запускает `camera_viewer`;
4. при `start_yolo:=true` запускает `yolo_detector`.

## PX4 SITL и модель `gz_x500_mono_cam`

PX4 SITL моделирует автопилот PX4 как процесс на компьютере. Target:

```text
px4_sitl gz_x500_mono_cam
```

показывает, что используется Gazebo-модель X500 с монокулярной камерой. Для
проекта это важно:

- миссионный узел получает состояние аппарата от PX4;
- камера Gazebo является источником изображения для YOLO;
- имена camera topics зависят от world/model/link/sensor;
- launch ищет модель, имя которой подходит под regex
  `x500_mono_cam(_[0-9]+)?`.

С точки зрения диплома PX4 SITL является имитационным стендом автопилота:
миссионный узел отправляет те же типы PX4-сообщений, но полет выполняется в
симуляционной среде.

## Роль ROS 2 и пакета `offboard_takeoff`

ROS 2 используется как прикладная шина данных проекта. В `setup.py` объявлены
console script-ы:

| Исполняемый файл | Назначение |
| --- | --- |
| `offboard_takeoff` | Главный миссионный узел `OffboardTakeoff`. |
| `camera_viewer` | Релей и просмотр изображения с камеры. |
| `yolo_detector` | Детектор людей, публикующий topic-и `yolo/*`. |
| `scene_map_publisher` | Публикация карты сцены для RViz. |
| `top_down_map_viewer` | OpenCV-карта сверху. |

`package.xml` фиксирует зависимости:

- `rclpy` для Python-узлов ROS 2;
- `px4_msgs` для обмена с PX4;
- `sensor_msgs` для `Image` и `CameraInfo`;
- `std_msgs` для bbox/topic-ов миссии;
- `visualization_msgs`, `geometry_msgs`, `tf2_ros` для карты и визуализации;
- `ros_gz_bridge` для моста Gazebo GZ -> ROS 2;
- `rosgraph_msgs` для `/clock`;
- `python3-numpy`, `python3-opencv` для обработки изображения и OpenCV-окон;
- `rviz2` для визуализации сцены.

## Роль `ros_gz_bridge`

`ros_gz_bridge` используется для передачи данных из Gazebo GZ в ROS 2 по
topic-ам камеры и времени. Создается node:

```text
package='ros_gz_bridge'
executable='parameter_bridge'
name='gz_camera_bridge'
```

Bridge получает аргументы:

```text
<gazebo_image_topic>@sensor_msgs/msg/Image[gz.msgs.Image
<gazebo_camera_info_topic>@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

Первые два topic-а определяются автоматически или задаются вручную через
launch-аргументы. `/clock` добавляется, если `bridge_clock:=true`.

Важно разделять два канала:

- управление PX4 идет через `MicroXRCEAgent` и `px4_msgs`;
- изображение камеры и симуляционное время идут через `ros_gz_bridge`.

В текущем launch `ros_gz_bridge` не мостит все topic-и Gazebo целиком. Он
настроен точечно под camera image, camera info и `/clock`.

## Auto-discovery Gazebo camera topics

Launch для person detection автоматически ищет реальные Gazebo camera topics.
Это нужно, потому что имя topic-а содержит world/model/link/sensor и может
отличаться между запусками.

Ожидаемая структура topic:

```text
^/world/(?P<world>[^/]+)/model/(?P<model>[^/]+)/
link/(?P<link>[^/]+)/sensor/(?P<sensor>[^/]+)/
(?P<kind>image|camera_info)$
```

Функция discovery выполняет:

```bash
gz topic -l
```

Затем она:

1. выбирает topic-и, подходящие под структуру Gazebo camera topic;
2. фильтрует их по имени модели через `model_regex`;
3. группирует пары `image` и `camera_info` по одному world/model/link/sensor;
4. выбирает лучшую пару по score;
5. возвращает `image_topic`, `camera_info_topic`, `camera_frame`,
   `world_name`, `model_name`.

Основные launch-аргументы discovery:

| Launch-аргумент | Значение по умолчанию | Смысл |
| --- | --- | --- |
| `model_regex` | `x500_mono_cam(_[0-9]+)?` | Какую модель Gazebo считать камерой дрона. |
| `topic_discovery_attempts` | `30` | Количество попыток выполнить поиск. |
| `topic_discovery_sleep_sec` | `0.5` | Пауза между попытками. |
| `image_topic` | пусто | Ручное переопределение Gazebo image topic. |
| `camera_info_topic` | пусто | Ручное переопределение Gazebo CameraInfo topic. |

Если `image_topic` и `camera_info_topic` заданы вручную, auto-discovery не
нужен. README приводит пример:

```bash
ros2 launch offboard_takeoff person_detection.launch.py \
  image_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image \
  camera_info_topic:=/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info
```

Ограничение: discovery требует, чтобы Gazebo уже был запущен и команда
`gz topic -l` видела camera topics. Поэтому порядок запуска важен: сначала
PX4/Gazebo, затем camera bridge launch.

## `camera_viewer` как релей изображения

После bridge ROS 2 получает Gazebo topic-и с длинными именами вида:

```text
/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

`camera_viewer` решает две задачи:

1. подписывается на найденные image/camera_info topics;
2. публикует стабильные relay topics:

```text
/camera/image_raw
/camera/camera_info
```

Он также может показывать OpenCV-окно и overlay последнего bbox из
`yolo/target_bbox`.

## YOLO node в симуляционном стеке

При `start_yolo:=true` запускается:

```text
package='offboard_takeoff'
executable='yolo_detector'
name='yolo_detector'
```

Основные параметры запуска из README:

```text
detector_architecture:=yolo
yolo_inference_backend:=ultralytics
yolo_input_width:=960
yolo_input_height:=960
```

YOLO node получает изображение, выполняет inference и публикует:

```text
yolo/target_detected
yolo/detection_count
yolo/class_ids
yolo/bounding_boxes
yolo/target_bbox
yolo/class_labels
yolo/debug_image
```

Ключевой topic для миссии:

```text
yolo/target_bbox
```

## `use_sim_time` и `/clock`

В симуляции ROS 2-узлам желательно использовать время Gazebo. В текущем
проекте это сделано через launch-параметры и конфиги:

- `bridge_clock` по умолчанию включает bridge для `/clock`;
- `use_sim_time` передается в `ros_gz_bridge`, `camera_viewer` и
  `yolo_detector`;
- `src/offboard_takeoff/config/yolo_detector.yaml` содержит
  `use_sim_time: true`.

Для карт ситуация отличается:

- `scene_map.launch.py` имеет `use_sim_time` по умолчанию `false` и
  комментирует, что `true` стоит включать только если `/clock` bridged;
- `top_down_map.launch.py` тоже имеет `use_sim_time` по умолчанию `false`.

## Что не запускается одним launch-файлом

В текущем репозитории запуск разделен:

1. PX4 SITL + Gazebo запускаются из `PX4-Autopilot`;
2. `MicroXRCEAgent` запускается отдельной командой;
3. camera bridge + viewer + YOLO запускаются через
   `person_detection.launch.py`;
4. миссионный узел `offboard_takeoff` запускается отдельно;
5. карта сцены и top-down viewer запускаются отдельными launch-файлами.

Это нормально для исследовательского стенда, но в дипломе нужно явно
перечислять порядок терминалов и параметры запуска.

## Gazebo CLI для телепорта модели

После завершения визуального действия с человеком миссионный узел может
телепортировать модель пострадавшего ниже сцены. В `node.py` используются:

```text
gz service -s /world/<world>/set_pose ...
gz topic -t /world/<world>/pose/modify ...
```

Список world-кандидатов включает значение `teleport_world_name`, затем
`forest_big`, `forest`, `default`, `empty`.

Это симуляционное действие, а не физическая эвакуация.

## Ограничения

1. Нет единого launch-файла для всего эксперимента.
2. Запуск зависит от локального окружения: `~/PX4-Autopilot`,
   `~/px4_offboard_clean_ws`, `/home/dron/.gz/models/...`.
3. Модель YOLO указана абсолютным локальным путем.
4. Camera auto-discovery зависит от того, что Gazebo уже запущен и `gz topic -l`
   возвращает нужные topics.
5. `scene_map.launch.py` по умолчанию использует `forest.sdf`, а
   `node.py` и `top_down_map.launch.py` используют `forest_big.sdf`. Для
   воспроизводимого эксперимента лучше явно выбрать один world-файл.
6. Телепорт модели через Gazebo CLI является частью симуляционного сценария.
7. Документ описывает запуск по коду и README, а не результаты проведенного
   эксперимента.

## Формулировка для диплома

Стенд моделирования построен на связке PX4 SITL, Gazebo GZ и ROS 2. PX4 SITL
моделирует автопилот квадрокоптера, Gazebo GZ предоставляет физическую и
визуальную среду с моделью `gz_x500_mono_cam`, а ROS 2-узлы реализуют
прикладную логику поиска, детекции людей, построения маршрутов и визуализации.
Связь PX4 с ROS 2 выполняется через `MicroXRCEAgent` и пакет `px4_msgs`, а
поток изображения и симуляционное время передаются из Gazebo в ROS 2 через
`ros_gz_bridge`.
