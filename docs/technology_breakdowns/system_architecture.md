# Общая архитектура системы

## Назначение

Данный документ описывает архитектуру дипломного проекта поисково-спасательного
квадрокоптера в симуляционной среде. Описание основано на текущих файлах
репозитория и предназначено для последующего использования при написании
технической главы диплома.

Проект реализует программный комплекс для автономного поиска человека с
использованием квадрокоптера PX4 в Gazebo GZ. Согласно `README.md`, система:

- запускает ROS 2 offboard-миссию для PX4;
- выполняет поиск по маршруту типа "змейка";
- получает изображения с камеры дрона через `ros_gz_bridge`;
- обнаруживает людей на изображении с помощью дообученной модели `YOLO12s`;
- публикует лучший bounding box человека в ROS 2;
- выполняет scripted visual response после подтверждения цели;
- может телепортировать эвакуированного человека из сцены Gazebo;
- продолжает поиск после эвакуации.

Ключевой смысл архитектуры заключается в разделении задачи на несколько
подсистем: симуляция и автопилот, ROS 2-управление, компьютерное зрение,
планирование маршрутов и визуализация состояния сцены.

## Состав системы

В репозитории выделяются два основных ROS 2-пакета:

- `src/offboard_takeoff` - основной прикладной пакет дипломного проекта:
  миссионная логика, управление PX4 в Offboard-режиме, детекторы, launch-файлы,
  обработка изображения, визуализация карты и планирование маршрутов.
- `src/px4_msgs` - пакет с ROS 2-описаниями сообщений PX4. Он используется
  миссионным узлом для обмена сообщениями с PX4 через topic-и `/fmu/in/*` и
  `/fmu/out/*`.

Симуляционная часть в репозитории не хранит сам PX4-Autopilot и Gazebo-мир как
исходники проекта. README предполагает, что PX4 запускается отдельно из
`~/PX4-Autopilot`, например командой:

```bash
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500_mono_cam
```

Также отдельно запускается `MicroXRCEAgent`:

```bash
MicroXRCEAgent udp4 -p 8888
```

Таким образом, рабочая система состоит из следующих внешних и внутренних
компонентов:

- PX4 SITL - программная симуляция автопилота;
- Gazebo GZ - физическая и визуальная среда, включая модель
  `gz_x500_mono_cam`;
- Micro XRCE-DDS Agent - мост между PX4 uORB/DDS и ROS 2;
- `ros_gz_bridge` - мост изображений, `CameraInfo` и `/clock` из Gazebo в
  ROS 2;
- ROS 2-узлы пакета `offboard_takeoff`;
- пакет `px4_msgs` с типами сообщений PX4.

## Роль пакета `src/offboard_takeoff`

Пакет `offboard_takeoff` является основной частью проекта. Его назначение
зафиксировано в `src/offboard_takeoff/package.xml`: "PX4 offboard mission
helpers and person detection for Gazebo SITL".

В `src/offboard_takeoff/setup.py` объявлены следующие console entry point-ы:

| Исполняемый узел | Python-модуль | Роль |
| --- | --- | --- |
| `offboard_takeoff` | `offboard_takeoff.offboard_takeoff:main` | Запуск миссионного узла `OffboardTakeoff` из `node.py`. |
| `yolo_detector` | `offboard_takeoff.yolo_detector:main` | Детекция людей на изображении и публикация результатов в topic-и `yolo/*`. |
| `camera_viewer` | `offboard_takeoff.camera_viewer:main` | Релей камеры, опциональное окно OpenCV и overlay bounding box. |
| `scene_map_publisher` | `offboard_takeoff.scene_map_publisher:main` | Публикация RViz-маркеров карты сцены, дрона, маршрута и людей. |
| `top_down_map_viewer` | `offboard_takeoff.top_down_map_viewer:main` | OpenCV-окно карты сверху с объектами сцены, траекторией и маршрутами. |

Основные исходные файлы:

- `src/offboard_takeoff/offboard_takeoff/node.py` - главный миссионный
  конечный автомат и PX4 Offboard-управление.
- `src/offboard_takeoff/offboard_takeoff/offboard_takeoff.py` - тонкий entry
  point, который создает и запускает `OffboardTakeoff`.
- `src/offboard_takeoff/offboard_takeoff/mission.py` - структуры `Waypoint`,
  `MissionPlan` и статический fallback-маршрут поиска.
- `src/offboard_takeoff/offboard_takeoff/navigation.py` - вспомогательные
  функции проверки достижения waypoint-а, нормализации yaw и сглаживания
  setpoint-а.
- `src/offboard_takeoff/offboard_takeoff/path_planner.py` - планировщик A* для
  эвакуационного маршрута и генератор поискового покрытия.
- `src/offboard_takeoff/offboard_takeoff/yolo_detector.py` - узел детекции
  объектов, публикующий совместимый интерфейс `yolo/*`.
- `src/offboard_takeoff/offboard_takeoff/camera_viewer.py` - релей и viewer
  изображения.
- `src/offboard_takeoff/offboard_takeoff/scene_map_publisher.py` -
  публикация карты в виде `visualization_msgs/MarkerArray`.
- `src/offboard_takeoff/offboard_takeoff/top_down_map_viewer.py` -
  независимое окно карты сверху.

## Роль пакета `src/px4_msgs`

Пакет `src/px4_msgs` содержит ROS 2 message/service definitions для PX4. Его
README описывает пакет как набор интерфейсов, необходимых для связи ROS 2-узлов
с внутренними сообщениями PX4.

В данном проекте миссионный узел импортирует из `px4_msgs.msg` следующие типы:

- `OffboardControlMode`;
- `TrajectorySetpoint`;
- `VehicleCommand`;
- `VehicleLocalPosition`;
- `VehicleStatus`.

Эти сообщения используются в `src/offboard_takeoff/offboard_takeoff/node.py`.
Через них ROS 2-узел отправляет команды в PX4 и получает статус автопилота.

## ROS 2-узлы

### Миссионный узел `offboard_takeoff`

Главный узел системы реализован классом `OffboardTakeoff` в
`src/offboard_takeoff/offboard_takeoff/node.py`. Он запускается через
`src/offboard_takeoff/offboard_takeoff/offboard_takeoff.py`.

Узел отвечает за:

- ожидание валидной локальной позиции PX4;
- сохранение home position;
- подготовку и публикацию Offboard setpoint-ов;
- arm и переключение PX4 в Offboard mode;
- взлет на рабочую высоту;
- движение по поисковому маршруту;
- прием лучшего bounding box человека из `yolo/target_bbox`;
- подтверждение цели;
- визуальное наведение на человека;
- построение эвакуационного маршрута A*;
- возврат домой или по эвакуационному маршруту;
- посадку или переход в failsafe;
- публикацию маршрутов для визуализации.

Состояния миссии заданы перечислением `MissionState` в `node.py`:

```text
INIT
WAIT_FOR_PX4
ARMING
TAKEOFF
SEARCH
HOLD
FOLLOW_PERSON
RETURN_HOME
LAND
FAILSAFE
FINISHED
```

Основной цикл управления реализован в методе `control_loop()`. Частота цикла
задается параметром `control_rate_hz`, значение по умолчанию в коде - `10.0`.

### Узел `camera_viewer`

Узел реализован в `src/offboard_takeoff/offboard_takeoff/camera_viewer.py`.
Он подписывается на изображение и `CameraInfo`, публикует их на стабильные
relay-topic-и и при необходимости показывает окно OpenCV.

По умолчанию параметры узла такие:

- входное изображение: `/camera/image_raw`;
- входной `CameraInfo`: `/camera/camera_info`;
- выходное изображение: `/camera/image_raw`;
- выходной `CameraInfo`: `/camera/camera_info`;
- topic bounding box для overlay: `yolo/target_bbox`.

В launch-файле эти параметры могут быть переопределены так, что узел читает
сырой Gazebo topic, а публикует стабильный ROS 2 topic `/camera/image_raw`.

### Узел `yolo_detector`

Узел реализован в `src/offboard_takeoff/offboard_takeoff/yolo_detector.py`.
Он получает кадры с камеры, выполняет inference и публикует результаты
детекции.

Из README и `src/offboard_takeoff/config/yolo_detector.yaml` следует, что
основной сценарий использует:

- модель `YOLO12s`;
- файл весов `best.pt` по пути
  `/home/dron/.gz/models/yolo12n_people_package/runs/yolo12s_people_e30_b62/weights/best.pt`;
- backend `ultralytics`;
- размер входа `960x960`;
- целевой класс `person`.

В рабочем сценарии используется YOLO-режим детектора. Для checkpoint `.pt`
основной backend - `ultralytics`; для ONNX-моделей в коде также предусмотрены
`onnxruntime` и fallback через OpenCV DNN.

Главный интерфейс для миссионной логики - topic:

```text
yolo/target_bbox
```

Формат сообщения `std_msgs/Int32MultiArray`:

```text
[confidence_milli, center_x, center_y, width, height, image_width, image_height]
```

Этот формат формируется в `yolo_detector.py` и разбирается в `node.py` в объект
`PersonTarget`.

### Узел `scene_map_publisher`

Узел реализован в `src/offboard_takeoff/offboard_takeoff/scene_map_publisher.py`.
Он загружает объекты из SDF-файла мира, подписывается на положение дрона и
публикует карту сцены в RViz как `visualization_msgs/MarkerArray`.

Основной публикуемый topic:

```text
scene_map/markers
```

Также узел подписывается на:

```text
/fmu/out/vehicle_local_position_v1
yolo/target_detected
mission/person_rescued
```

По текущему коду `scene_map_publisher` может скрывать/телепортировать
спасенного человека при получении события `mission/person_rescued`. Однако в
текущем `node.py` publisher этого события не найден. Это расхождение подробно
отмечено в разделе "Ограничения и замечания".

### Узел `top_down_map_viewer`

Узел реализован в `src/offboard_takeoff/offboard_takeoff/top_down_map_viewer.py`.
Он строит OpenCV-представление карты сверху. В отличие от RViz-карты, это
локальное окно с отрисовкой объектов мира, положения БПЛА и маршрутов.

Узел подписывается на:

```text
/fmu/out/vehicle_local_position_v1
mission/search_route
mission/evacuation_route
```

Таким образом, `top_down_map_viewer` является потребителем маршрутов,
публикуемых миссионным узлом.

## Launch-файлы

### `person_detection.launch.py`

Файл `src/offboard_takeoff/launch/person_detection.launch.py` является
предпочтительной точкой запуска camera bridge и person detection. В рабочем
сценарии этот запуск:

- опрашивает `gz topic -l`;
- автоматически ищет пару Gazebo topic-ов `image` и `camera_info` для модели,
  соответствующей regex `x500_mono_cam(_[0-9]+)?`;
- запускает `ros_gz_bridge` для изображения, `CameraInfo` и, опционально,
  `/clock`;
- запускает `camera_viewer`;
- при `start_yolo:=true` запускает `yolo_detector`.

Типичные relay-topic-и:

```text
/camera/image_raw
/camera/camera_info
```

Launch-файл также допускает ручное указание topic-ов через `image_topic` и
`camera_info_topic`.

### `scene_map.launch.py`

Файл `src/offboard_takeoff/launch/scene_map.launch.py` запускает:

- `scene_map_publisher`;
- RViz2, если `start_rviz:=true`;
- `top_down_map_viewer`, если `start_top_down_viewer:=true`.

По умолчанию путь к миру в этом launch-файле:

```text
/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf
```

### `top_down_map.launch.py`

Файл `src/offboard_takeoff/launch/top_down_map.launch.py` запускает только
`top_down_map_viewer`.

По умолчанию путь к миру:

```text
/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf
```

## Основные topic-и и потоки данных

### Поток управления PX4

Миссионный узел `OffboardTakeoff` публикует:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Назначение:

- `/fmu/in/offboard_control_mode` - указание PX4, что внешний контроллер
  управляет позицией;
- `/fmu/in/trajectory_setpoint` - целевая позиция и yaw в локальной системе
  координат PX4;
- `/fmu/in/vehicle_command` - команды arm, переключение режима и посадка.

Миссионный узел подписывается на:

```text
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v2
```

Назначение:

- `/fmu/out/vehicle_local_position_v1` - текущая локальная позиция, скорость и
  yaw БПЛА;
- `/fmu/out/vehicle_status_v2` - состояние автопилота, включая armed/offboard
  признаки, используемые при переходах состояний.

QoS для PX4 topic-ов в `node.py`, `scene_map_publisher.py` и
`top_down_map_viewer.py` настроен как `BEST_EFFORT`, `TRANSIENT_LOCAL`,
`KEEP_LAST`, глубина `1`.

### Поток изображения

Gazebo публикует собственные topic-и камеры вида:

```text
/world/<world>/model/<model>/link/<link>/sensor/<sensor>/image
/world/<world>/model/<model>/link/<link>/sensor/<sensor>/camera_info
```

Launch для person detection находит эти topic-и через `gz topic -l` и
запускает `ros_gz_bridge`. Далее `camera_viewer` может переиздавать поток в
стабильные ROS 2 topic-и:

```text
/camera/image_raw
/camera/camera_info
```

Детектор читает изображение из параметра `image_topic`. В конфигурации
`src/offboard_takeoff/config/yolo_detector.yaml` это `/camera/image_raw`.

### Поток компьютерного зрения

`yolo_detector` публикует набор topic-ов:

```text
yolo/debug_image
yolo/target_detected
yolo/detection_count
yolo/class_ids
yolo/bounding_boxes
yolo/target_bbox
yolo/class_labels
```

Для миссионного конечного автомата главным является:

```text
yolo/target_bbox
```

Для карты сцены используется:

```text
yolo/target_detected
```

`camera_viewer` также может подписываться на `yolo/target_bbox`, чтобы
отрисовывать overlay на изображении.

### Поток маршрутов миссии

Миссионный узел публикует:

```text
mission/search_route
mission/evacuation_route
```

Оба topic-а имеют тип `std_msgs/Float32MultiArray`.

`mission/search_route` публикуется для визуализации поискового маршрута.
`mission/evacuation_route` публикуется после построения A*-маршрута возврата.

`top_down_map_viewer` подписывается на оба topic-а и отрисовывает их на карте
сверху.

### Поток карты сцены

`scene_map_publisher` публикует:

```text
scene_map/markers
```

Этот topic предназначен для RViz и содержит маркеры объектов сцены, маршрута,
дрона, следа полета, точек обнаружений и людей, если они были загружены из
SDF-файла.

## Связь PX4, Gazebo и ROS 2

В проекте используется две разные интеграционные связи:

1. PX4 <-> ROS 2 через Micro XRCE-DDS Agent и `px4_msgs`.
2. Gazebo GZ <-> ROS 2 через `ros_gz_bridge`.

PX4 SITL и Gazebo запускаются вне данного workspace. После запуска PX4 SITL
модель `gz_x500_mono_cam` существует в Gazebo-мире и публикует данные камеры.
`ros_gz_bridge` переводит Gazebo-сообщения изображения и `CameraInfo` в ROS 2.

Параллельно Micro XRCE-DDS Agent обеспечивает связь PX4 с ROS 2 topic-ами
`/fmu/in/*` и `/fmu/out/*`. Благодаря этому миссионный узел может публиковать
`TrajectorySetpoint` и получать `VehicleLocalPosition`.

С точки зрения проекта, эти две связи используются независимо:

- изображение от Gazebo поступает в detector node;
- состояние и команды автопилота идут через PX4 topic-и;
- объединение потоков происходит в `OffboardTakeoff`, который принимает
  `yolo/target_bbox` и состояние PX4, а затем меняет setpoint-ы для БПЛА.

## Планирование маршрутов

Планирование реализовано в `src/offboard_takeoff/offboard_takeoff/path_planner.py`.

### Поисковый маршрут

Если параметр `enable_auto_search_route` равен `True`, миссионный узел пытается
построить маршрут покрытия через `SearchCoveragePlanner`.

Параметры по умолчанию в `node.py`:

- `search_world_sdf_path`:
  `/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf`;
- `search_grid_resolution`: `1.0`;
- `search_obstacle_margin`: `1.5`;
- `search_map_padding`: `4.0`;
- `search_row_spacing`: `7.0`;
- `search_waypoint_spacing`: `3.0`;
- `search_hold_time_sec`: `1.5`;
- `search_mission_rotation_deg`: `90.0`;
- `search_mission_invert_y`: `True`.

`SearchCoveragePlanner` наследуется от `EvacuationRoutePlanner`. Он загружает
препятствия из SDF, строит строки покрытия свободного пространства, находит
свободные интервалы на строках и соединяет сегменты через A* или прямым
соединением, если линия свободна.

Если автоматический маршрут не был построен, `node.py` использует fallback из
`src/offboard_takeoff/offboard_takeoff/mission.py`: статический маршрут
"змейкой" вокруг forest search area.

### Эвакуационный маршрут

После завершения визуального взаимодействия с человеком миссионный узел
готовит возврат через `_prepare_evacuation_route()` в `node.py`.

Если `enable_evacuation_astar` равен `True`, используется
`EvacuationRoutePlanner`. Параметры по умолчанию:

- `evacuation_world_sdf_path`:
  `/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf`;
- `evacuation_grid_resolution`: `1.0`;
- `evacuation_obstacle_margin`: `3.0`;
- `evacuation_map_padding`: `8.0`;
- `evacuation_waypoint_spacing`: `3.0`;
- `evacuation_mission_rotation_deg`: `90.0`;
- `evacuation_mission_invert_y`: `True`;
- `evacuation_max_iterations`: `20000`.

Планировщик читает SDF, выделяет известные типы препятствий по именам моделей,
строит inflated obstacle polygons в mission-local XY и ищет путь по сетке A*.
Люди, могилы и некоторые похожие сущности исключаются из списка препятствий
через `EXCLUDED_MODEL_NAMES`.

## Последовательность работы системы

Ниже приведена последовательность работы по README и текущему коду без
утверждения, что она была экспериментально проверена в рамках подготовки этого
документа.

1. Запускается PX4 SITL с Gazebo-моделью `gz_x500_mono_cam`.
2. Запускается `MicroXRCEAgent udp4 -p 8888`.
3. Запускается launch для камеры и детекции:

   ```bash
   ros2 launch offboard_takeoff person_detection.launch.py start_yolo:=true
   ```

4. Launch-файл ищет Gazebo camera topic-и, запускает `ros_gz_bridge`, запускает
   `camera_viewer` и при необходимости `yolo_detector`.
5. `camera_viewer` переиздает поток камеры на стабильный topic
   `/camera/image_raw`.
6. `yolo_detector` читает изображение, выполняет inference и публикует
   `yolo/target_bbox`.
7. Отдельно запускается миссионный узел `offboard_takeoff`.
8. `OffboardTakeoff` ожидает валидную локальную позицию PX4 и сохраняет home
   position.
9. Узел отправляет 30 начальных Offboard setpoint-ов. При частоте 10 Гц это
   около 3 секунд.
10. Узел отправляет команду перехода в Offboard mode и arm-команду.
11. После подтверждения состояния или fallback-таймаута миссия переходит к
    взлету.
12. БПЛА достигает рабочей высоты, заданной `takeoff_height`.
13. Миссия начинает поиск по маршруту `SEARCH`.
14. Во время `SEARCH` и `HOLD` узел проверяет свежий `yolo/target_bbox`.
15. Если человек подтвержден в течение `follow_detection_confirm_sec`, миссия
    переходит в `FOLLOW_PERSON`.
16. В `FOLLOW_PERSON` выполняются фазы:
    `STOP_BEFORE_APPROACH`, `ALIGN_TO_TARGET`, `APPROACH_FORWARD`,
    `HOLD_AFTER_APPROACH`.
17. После завершения визуального подлета узел готовит маршрут возврата и
    переходит в `RETURN_HOME`.
18. Если A*-маршрут найден, возврат идет по его waypoint-ам; иначе используется
    прямой возврат к home position.
19. После возврата домой код пытается телепортировать настроенную модель
    `rescue_model_name` вниз/из сцены через `gz service` или `gz topic`.
20. Затем включается cooldown детекции и миссия возобновляет поиск с сохраненной
    позиции маршрута.
21. Когда поисковый маршрут завершается, миссия возвращается домой и переходит
    к посадке, если `auto_land_on_finish=True`.

## Ограничения и допущения

### Документ описывает код, а не результаты испытаний

В данном файле не утверждается, что система была запущена или проверена
экспериментально при подготовке документа. Описание основано на чтении
`README.md`, файлов `docs` и исходного кода репозитория.

### Расхождение по `mission/person_rescued`

README описывает, что миссионный узел публикует событие:

```text
mission/person_rescued
```

`scene_map_publisher.py` действительно подписывается на этот topic и содержит
логику обработки спасенного человека.

Однако в текущем `src/offboard_takeoff/offboard_takeoff/node.py` найденные
publisher-ы миссионного узла:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
mission/evacuation_route
mission/search_route
```

Publisher для `mission/person_rescued` в текущем `node.py` не найден. Вместо
этого после возврата домой миссионный узел сам вызывает телепортацию
настроенной модели через `gz service` или `gz topic`.

Следовательно, описание README и текущая реализация отличаются: архитектура
карты сцены ожидает событие спасения, но миссионный узел в текущем состоянии
не публикует это событие.

### Расхождение по параметру `resume_search_after_rescue`

README перечисляет параметр:

```text
resume_search_after_rescue
```

В текущих параметрах `OffboardTakeoff` в `node.py` такого параметра не найдено.
Продолжение поиска реализовано внутренними флагами:

```text
pending_search_resume_after_home
resume_search_waypoint_index
pending_teleport_after_home
```

То есть возобновление поиска после возврата домой реализовано в коде, но не
через публичный ROS 2-параметр с именем `resume_search_after_rescue`.

### Разные значения `takeoff_height` в документации

В `docs/technology_breakdowns/mission_state_machine.md` указано значение
`takeoff_height = 5.0`. В текущем `node.py` значение параметра по умолчанию:

```text
takeoff_height = 7.0
```

В `scene_map_publisher.py` значение по умолчанию для карты:

```text
takeoff_height = 5.0
```

Следовательно, при описании диплома нужно аккуратно разделять параметры
миссионного узла и параметры визуализации, либо привести их к одному значению в
коде и документации.

### Разные SDF-миры в launch-файлах

В миссионном узле и `top_down_map.launch.py` по умолчанию используется:

```text
forest_big.sdf
```

В `scene_map.launch.py` по умолчанию указан:

```text
forest.sdf
```

Это может быть осознанным выбором для разных визуализаций, но для дипломного
описания лучше явно указать, какой мир использовался в основной демонстрации.

### Эвакуация является симуляционным действием

Код не реализует физический захват или транспортировку человека. После
визуального взаимодействия с целью он возвращает БПЛА домой, а затем пытается
телепортировать настроенную модель `rescue_model_name` в Gazebo.

В `node.py` по умолчанию:

```text
rescue_model_name = man_3
rescue_model_teleport_x = 36.57
rescue_model_teleport_y = 28.2
rescue_model_teleport_z = -10.0
teleport_world_name = forest_big
```

Поэтому в дипломе корректнее писать о "симуляционной отметке эвакуации" или
"моделировании эвакуации через перенос объекта сцены", а не о физической
эвакуации пострадавшего.

### Дальность до человека напрямую не измеряется

В текущей логике наведения дальность до человека не получается из depth-камеры
или лидара. Подлет завершается по косвенным признакам из изображения:

- рост высоты bounding box относительно начального размера;
- достижение нижней границы кадра;
- таймаут движения вперед;
- потеря цели в течение заданного времени.

Эта часть подробнее описана в
`docs/technology_breakdowns/target_acquisition.md`.

### Планировщик зависит от SDF и известных размеров моделей

`path_planner.py` строит препятствия из SDF include-узлов и словаря
`MODEL_FOOTPRINTS`. Если в Gazebo-мире есть объект, имя которого не попадает в
этот словарь, планировщик может не учитывать его как препятствие.

## Краткая формулировка для диплома

Программная архитектура проекта построена как ROS 2-система, интегрированная с
PX4 SITL и Gazebo GZ. PX4 выполняет роль автопилота квадрокоптера, Gazebo
моделирует среду и камеру, а ROS 2-узлы реализуют миссионную логику,
компьютерное зрение, планирование маршрутов и визуализацию. Связь с PX4
осуществляется через Micro XRCE-DDS Agent и сообщения пакета `px4_msgs`, а
изображение с камеры передается из Gazebo в ROS 2 через `ros_gz_bridge`.

Центральным компонентом является миссионный узел `OffboardTakeoff`, реализующий
конечный автомат автономного полета. Узел публикует Offboard setpoint-ы в PX4,
принимает состояние автопилота и реагирует на bounding box человека,
публикуемый detector node в topic `yolo/target_bbox`. Поисковый маршрут
формируется как покрытие области типа "змейка" с учетом препятствий из SDF, а
возврат после обнаружения цели может строиться A*-планировщиком. Визуализация
состояния системы выполняется через RViz-маркеры и отдельное OpenCV-окно карты
сверху.
