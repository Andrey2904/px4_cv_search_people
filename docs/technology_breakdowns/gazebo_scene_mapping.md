# Карта сцены Gazebo, SDF-парсинг и визуализация

Этот документ описывает, как в проекте строится карта симуляционной сцены,
как из SDF-файла Gazebo извлекаются объекты, как они отображаются в RViz и
OpenCV, и как поверх карты показываются маршруты поиска, маршрут возврата,
положение квадрокоптера и события обнаружения человека.

Текст написан как технический материал для диплома. Ниже описана только
текущая реализация, без предположений о том, чего в коде нет.

## Где реализовано

Основные файлы:

- `src/offboard_takeoff/offboard_takeoff/scene_map_publisher.py` - ROS 2-узел
  для публикации RViz-маркеров карты сцены.
- `src/offboard_takeoff/offboard_takeoff/top_down_map_viewer.py` - ROS 2-узел
  с отдельным OpenCV-окном карты сверху.
- `src/offboard_takeoff/offboard_takeoff/path_planner.py` - загрузка
  препятствий из SDF для A* и покрытия зоны поиска.
- `src/offboard_takeoff/launch/scene_map.launch.py` - запуск RViz-карты,
  опционально вместе с OpenCV-viewer.
- `src/offboard_takeoff/launch/top_down_map.launch.py` - запуск только
  OpenCV-viewer.
- `README.md` и документы в `docs/technology_breakdowns/` - описание
  архитектуры, маршрутов, миссии и зафиксированных расхождений README с кодом.

## Назначение карты сцены

Карта сцены выполняет три разные функции.

Первая функция - визуализация среды. Узлы карты показывают статические объекты
из Gazebo world-файла: завалы, здания, дома, деревья и другие модели, которые
подходят под заданные ключевые слова или успешно распознаны как препятствия.

Вторая функция - визуализация состояния миссии. На карту накладываются
поисковый маршрут, текущая позиция квадрокоптера, направление его движения,
история траектории, точки обнаружения цели и, в OpenCV-viewer, маршрут
эвакуации/возврата, построенный A*.

Третья функция - использование той же SDF-сцены как априорной карты для
планирования. В `path_planner.py` SDF-файл читается не для рисования, а для
построения двумерных footprint-ов препятствий. Эти footprint-ы затем
используются в A* и при генерации маршрута покрытия зоны поиска.

## SDF world как источник карты

В проекте Gazebo world-файл формата SDF выступает априорным описанием сцены.
В нем хранятся включенные модели:

```xml
<world>
  <include>
    <uri>model://...</uri>
    <name>...</name>
    <pose>x y z roll pitch yaw</pose>
  </include>
</world>
```

И `scene_map_publisher.py`, и `top_down_map_viewer.py`, и `path_planner.py`
читают именно такие `<include>`-элементы внутри корневого `<world>`.

Общая схема чтения:

1. Берется путь из параметра `world_sdf_path`.
2. Файл читается как XML.
3. Перед парсингом исправляется известный вариант некорректного комментария:
   `<!---` заменяется на `<!--`, а `--->` на `-->`.
4. XML парсится через `xml.etree.ElementTree`.
5. Из корневого XML ищется элемент `<world>`.
6. Из каждого `<include>` берутся `uri`, `name` и `pose`.
7. Строка `pose` разбирается как `x y z roll pitch yaw`; если значения
   отсутствуют или не парсятся, используется `0.0`.

Если SDF-файл не найден, XML не парсится или `<world>` отсутствует, узлы не
строят карту по объектам. Визуализатор при этом не создает фиктивных объектов:
он либо возвращает пустой список, либо строит минимальные границы карты.

## Два вида визуализации

В проекте есть две независимые визуализации карты, которые решают разные
задачи.

### SceneMapPublisher и RViz-маркеры

`SceneMapPublisher` - это ROS 2-узел `scene_map_publisher`. Он публикует
`visualization_msgs/MarkerArray` в topic:

```text
scene_map/markers
```

Этот topic предназначен для RViz. Узел также публикует статический TF между
`parent_frame_id` и `frame_id`, если эти frame-ы различаются. По умолчанию:

```text
parent_frame_id = map
frame_id = mission_map
```

Статический transform является identity-transform: он нужен не для сложного
геометрического преобразования, а чтобы у RViz был валидный fixed frame для
маркеров.

`SceneMapPublisher` отображает:

- статические объекты сцены как `Marker.CUBE`;
- подписи объектов как `Marker.TEXT_VIEW_FACING`;
- людей как красные `Marker.CYLINDER`;
- подписи людей;
- маршрут поиска из `default_mission_plan()`;
- точки поисковых waypoint-ов;
- прямоугольник области поиска;
- home/takeoff-точку;
- trail квадрокоптера;
- точки, в которых произошел rising edge `yolo/target_detected`;
- текущую позу квадрокоптера как `Marker.ARROW`.

Важно: `SceneMapPublisher` не подписывается на `mission/search_route` и
`mission/evacuation_route`. Поэтому в текущем виде он рисует не динамически
сгенерированный маршрут поиска и не A*-маршрут, а маршрут из
`default_mission_plan(self.takeoff_height)`. Динамические маршруты отображает
OpenCV-viewer.

### TopDownMapViewer и OpenCV

`TopDownMapViewer` - это ROS 2-узел `top_down_map_viewer`. Он не публикует
маркеры в RViz, а рисует карту в отдельном окне OpenCV:

```text
window_name = Gazebo Top-Down Map
window_width = 1100
window_height = 850
```

OpenCV-viewer отображает:

- границы карты;
- сетку с шагом 5 м;
- оси X/Y, если `draw_axes=True`;
- статические неперсональные объекты;
- подписи объектов, если `draw_object_labels=True`;
- маршрут поиска из `mission/search_route`;
- A*-маршрут из `mission/evacuation_route`;
- текущую позицию квадрокоптера и направление heading.

В отличие от RViz-визуализации, OpenCV-viewer является удобным компактным
инструментом для просмотра миссии сверху. Он показывает именно маршруты,
которые публикуются миссионным узлом в ROS 2 topic-и.

При ошибке `cv2.imshow()` узел выставляет `window_available=False` и больше не
пытается рисовать окно. Это важно для запуска без GUI-доступа: в такой среде
узел не сможет показать OpenCV-окно.

## Отличия RViz-карты и OpenCV-viewer

| Свойство | `scene_map_publisher.py` | `top_down_map_viewer.py` |
|---|---|---|
| Тип вывода | ROS 2 `MarkerArray` для RViz | OpenCV-окно |
| Основной topic | `scene_map/markers` | Нет выходного topic-а |
| Статический TF | Да, `map -> mission_map` | Нет |
| Объекты сцены | Да | Да |
| Люди | Да, отдельные красные цилиндры | Нет, люди исключаются |
| Поисковый маршрут | Статический `default_mission_plan()` | Динамический `mission/search_route` |
| A*-маршрут | Нет | Да, `mission/evacuation_route` |
| Trail дрона | Да | Нет |
| Точки detection | Да | Нет |
| Текущий дрон | Да, стрелка | Да, круг и линия heading |
| Rescue-событие | Подписка на `mission/person_rescued` | Нет |

Для диплома это можно формулировать так: в системе реализованы две
визуализации одной симуляционной сцены. RViz-визуализация больше подходит для
интеграции в ROS 2-инструменты и отображения дополнительных маркеров миссии, а
OpenCV-viewer служит легким обзорным окном для контроля карты, маршрутов и
текущего положения БПЛА.

## Запуск через launch-файлы

`scene_map.launch.py` запускает:

- `scene_map_publisher`;
- `rviz2`, если `start_rviz:=true`;
- `top_down_map_viewer`, если `start_top_down_viewer:=true`.

Значение `world_sdf_path` в этом launch-файле по умолчанию:

```text
/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf
```

`top_down_map.launch.py` запускает только `top_down_map_viewer`. Значение
`world_sdf_path` по умолчанию:

```text
/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf
```

Это важное расхождение: если запускать RViz-карту и OpenCV-viewer разными
launch-файлами без явного `world_sdf_path`, они могут читать разные SDF-миры.
Для корректного сравнения карты, маршрутов и реальной Gazebo-сцены путь к
world-файлу лучше задавать явно.

Пример запуска RViz-карты:

```bash
ros2 launch offboard_takeoff scene_map.launch.py \
  world_sdf_path:=/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf
```

Пример запуска OpenCV-карты:

```bash
ros2 launch offboard_takeoff top_down_map.launch.py \
  world_sdf_path:=/home/dron/PX4-Autopilot/Tools/simulation/gz/worlds/forest_big.sdf
```

## Классификация объектов сцены

### Классификация в SceneMapPublisher

`SceneMapPublisher` делит объекты SDF на две группы:

- `scene_objects` - статические объекты/завалы;
- `person_objects` - модели людей.

Для этого используется поиск ключевых слов в строке:

```text
<resolved_name> <uri>
```

Перед сравнением строка приводится к нижнему регистру, символ `_` заменяется
на пробел, затем текст разбивается на токены по неалфавитно-цифровым
символам.

Ключевые слова для статических объектов по умолчанию:

```text
rubble
destroy
debris
collapsed
ruin
wreck
house
building
mango_tree
```

Ключевые слова для людей по умолчанию:

```text
man
men
person
people
victim
walker
```

Если модель не совпала ни с одной из групп, `SceneMapPublisher` ее не
отображает. То есть RViz-карта не является полным рендером всего SDF world:
она показывает только те include-модели, которые попали под заданные
категории.

### Классификация в TopDownMapViewer

`TopDownMapViewer` отображает только неперсональные объекты. Он исключает:

- модели, имя которых входит в `excluded_model_names`;
- модели, распознанные как люди по `person_keywords`.

Значения по умолчанию:

```text
excluded_model_names = ['grave_1']
person_keywords = ['man', 'men', 'person', 'people', 'victim', 'walker']
```

В отличие от `SceneMapPublisher`, OpenCV-viewer не рисует людей отдельным
слоем. Это сделано явно: люди исключаются из списка объектов карты.

### Классификация в PathPlanner

`path_planner.py` использует SDF не для рисования всех объектов, а для
построения препятствий. Препятствием становится только та модель, имя которой
есть в словаре `MODEL_FOOTPRINTS`.

Текущий словарь:

```python
MODEL_FOOTPRINTS = {
    'house': (16.27, 6.07),
    'destroy_building': (30.58, 28.97),
    'roof_rubble': (3.015, 3.015),
    'rubble': (10.79, 4.09),
    'mango_tree': (4.225, 4.225),
}
```

Модели людей исключаются через `EXCLUDED_MODEL_NAMES`:

```text
grave
man
men
person
people
victim
walker
```

Также явно исключается `grave`. Если модель не входит в `MODEL_FOOTPRINTS`,
она игнорируется планировщиком как препятствие.

## Обработка моделей людей

Люди в проекте обрабатываются по-разному в разных подсистемах.

В `SceneMapPublisher` люди:

- загружаются из SDF в список `person_objects`;
- отображаются в RViz красными цилиндрами;
- получают подписи с именем модели;
- могут быть скрыты после события `mission/person_rescued`;
- могут быть телепортированы из сцены через Gazebo service.

В `TopDownMapViewer` люди:

- распознаются по ключевым словам;
- исключаются из карты объектов;
- не отображаются отдельными маркерами.

В `path_planner.py` люди:

- исключаются из препятствий;
- не влияют на A* и маршрут покрытия;
- не задают занятые клетки сетки.

Это честное ограничение текущей реализации: планировщик считает статическими
препятствиями только завалы/постройки/деревья из `MODEL_FOOTPRINTS`, но не
учитывает людей как динамические препятствия или цели.

## Размеры объектов и footprint corrections

### Размеры в SceneMapPublisher

RViz-карта использует ручные размеры объектов, заданные параметрами. Значения
по умолчанию:

| Параметр | Значение | Назначение |
|---|---:|---|
| `object_size_xy` | `3.5` | XY-размер объекта по умолчанию |
| `object_size_z` | `2.0` | Высота объекта по умолчанию |
| `house_size_x` | `16.27` | Длина модели `house` |
| `house_size_y` | `6.07` | Ширина модели `house` |
| `house_size_z` | `9.90` | Высота модели `house` |
| `building_size_x` | `30.58` | Длина `destroy_building` |
| `building_size_y` | `28.97` | Ширина `destroy_building` |
| `building_size_z` | `8.18` | Высота `destroy_building` |
| `roof_rubble_size_xy` | `6.03` | XY-размер `roof_rubble` |
| `roof_rubble_size_z` | `6.72` | Высота `roof_rubble` |
| `rubble_size_x` | `10.79` | Длина `rubble` |
| `rubble_size_y` | `4.09` | Ширина `rubble` |
| `rubble_size_z` | `12.22` | Высота `rubble` |

Для людей `SceneMapPublisher` рисует не полный footprint модели, а цилиндр:

```text
scale.x = 0.8
scale.y = 0.8
scale.z = max(scene_object.size_z, 1.6)
```

Таким образом, RViz-слой людей является условным маркером положения человека,
а не точной геометрией модели.

### Размеры в TopDownMapViewer

OpenCV-viewer пытается определить размеры объектов более автоматически.

Порядок определения размера:

1. Из `model://...` извлекается имя модели.
2. Поиск `model.sdf` идет в `model_search_paths`.
3. Сначала читаются geometry из `.//collision/geometry`.
4. Для модели `house` дополнительно читаются mesh-geometry из
   `.//visual/geometry`.
5. Если collision-geometry не дала размеров, читаются visual-geometry.
6. Если размер не найден, используется `object_fallback_size_m`.

Пути поиска моделей по умолчанию:

```text
/home/dron/.gz/models
/home/dron/.gz/models/model_ruble
/home/dron/.gz/models/model_people
/home/dron/PX4-Autopilot/Tools/simulation/gz/models
```

Поддерживаемые типы geometry:

- `box/size`;
- `cylinder/radius`;
- `sphere/radius`;
- mesh только для `.gltf`, если размеры можно оценить по accessor
  `min`/`max` внутри GLTF.

Если найдено несколько размеров, выбирается тот, у которого максимальная
площадь `size_x * size_y`.

После этого применяются correction-правила, которые нужны только для
отрисовки карты:

| Модель | Коррекция |
|---|---|
| `destroy_building` | `size_x / 2`, `size_y / 2` |
| `roof_rubble` | квадрат `max(size_x, size_y) * 0.5` |
| `rubble` | если `size_x >= size_y`, уменьшается `size_x / 2`, иначе `size_y / 2` |
| `mango_tree` | оба размера умножаются на `0.625` |

Эти correction-правила не доказывают физический размер модели. Они являются
практической подгонкой footprint-а для читаемой карты сверху.

### Размеры в PathPlanner

Планировщик не читает `model.sdf`, collision geometry или mesh-файлы. Он
использует только ручной словарь `MODEL_FOOTPRINTS`.

После выбора footprint-а препятствие расширяется на `obstacle_margin`.
Значение по умолчанию:

```text
obstacle_margin = 3.0 м
```

Для каждого препятствия строится повернутый прямоугольник:

1. Берется центр из SDF `pose.x`, `pose.y`.
2. Берется yaw из SDF `pose.yaw`.
3. Половина размера увеличивается на `obstacle_margin`.
4. Четыре локальных угла поворачиваются по yaw.
5. Полученные world-точки переводятся в mission-local координаты.

Именно эти расширенные прямоугольники используются в проверке занятости
клеток A* и при построении свободных интервалов для покрытия зоны поиска.

## Преобразования координат

В проекте одновременно используются несколько двумерных систем координат:

- Gazebo world XY - координаты объектов в SDF world-файле.
- Mission-local XY - локальные координаты PX4/миссии, в которых задаются
  waypoint-ы и маршруты.
- Map XY - координаты карты для RViz или OpenCV.
- Pixel XY - координаты изображения OpenCV-окна.

### Gazebo world XY -> map XY

И `SceneMapPublisher`, и `TopDownMapViewer` имеют функцию
`_world_to_map_xy()`. Она применяет одинаковую идею преобразования:

1. Опционально поменять X и Y местами: `world_swap_xy`.
2. Опционально инвертировать X: `world_invert_x`.
3. Опционально инвертировать Y: `world_invert_y`.
4. Повернуть координаты на `world_rotation_deg`.
5. Добавить `world_offset_x`, `world_offset_y`.

По умолчанию эти параметры в обоих узлах равны:

```text
world_swap_xy = False
world_invert_x = False
world_invert_y = False
world_rotation_deg = 0.0
world_offset_x = 0.0
world_offset_y = 0.0
```

То есть по умолчанию Gazebo world XY совпадает с map XY.

### Mission-local XY -> RViz map XY

В `SceneMapPublisher` миссионные координаты переводятся в карту методом
`_mission_to_map_xy()`:

```text
rotation = mission_rotation_deg
offset = mission_offset_x, mission_offset_y
```

Параметры по умолчанию:

```text
mission_rotation_deg = 90.0
mission_offset_x = 0.0
mission_offset_y = 0.0
```

В этой реализации для RViz нет параметров `mission_swap_xy`,
`mission_invert_x`, `mission_invert_y`. Есть только поворот и смещение.

### Mission-local XY -> OpenCV map XY

В `TopDownMapViewer` преобразование mission-local координат богаче:

1. Опционально поменять X и Y местами: `mission_swap_xy`.
2. Опционально инвертировать X: `mission_invert_x`.
3. Опционально инвертировать Y: `mission_invert_y`.
4. Повернуть на `mission_rotation_deg`.
5. Добавить `mission_offset_x`, `mission_offset_y`.

Значения по умолчанию:

```text
mission_swap_xy = False
mission_invert_x = False
mission_invert_y = True
mission_rotation_deg = 90.0
mission_offset_x = 0.0
mission_offset_y = 0.0
```

Это означает, что OpenCV-viewer по умолчанию не просто поворачивает
mission-local координаты, но и инвертирует Y перед поворотом. Для совпадения
слоев карты, маршрутов и позиции дрона эти параметры должны соответствовать
тому, как миссионные координаты соотнесены с Gazebo world.

### Map XY -> pixel XY в OpenCV

После перевода всех объектов в map XY OpenCV-viewer переводит метры в пиксели:

1. По объектам вычисляются границы карты.
2. К ним добавляется `map_padding_m`.
3. Если ширина или высота меньше `min_map_size_m`, диапазон расширяется.
4. Масштаб выбирается так, чтобы карта поместилась в окно с полями 80 пикселей.
5. X растет вправо.
6. Y инвертируется для изображения: большее map-Y находится выше на экране.

Таким образом, OpenCV-окно показывает обычный вид сверху, но внутри использует
пиксельную систему координат изображения, где ось Y направлена вниз.

### Преобразование в PathPlanner

`path_planner.py` строит препятствия в mission-local XY. Для этого он берет
координаты из SDF и применяет `_map_to_mission_xy()`, который описан в коде как
обратное преобразование к mission-to-map transform, используемому viewer-ом.

Конфигурация планировщика:

```python
PlannerConfig(
    mission_rotation_deg=90.0,
    mission_invert_y=True,
)
```

При этом `path_planner.py` не содержит параметров `world_swap_xy`,
`world_invert_x`, `world_invert_y`, `world_rotation_deg` и world-offset. То
есть он предполагает, что координаты SDF уже находятся в той map-системе, для
которой надо выполнить обратное преобразование в mission-local XY.

Это ограничение важно: если для визуализации используются нестандартные
world-преобразования, планировщик сам по себе их не повторяет.

## Отображение маршрута поиска

В текущем проекте есть два варианта отображения поискового маршрута.

В RViz через `SceneMapPublisher` отображается маршрут из
`default_mission_plan(self.takeoff_height)`. Узел строит:

- `mission_path` как `Marker.LINE_STRIP`;
- `mission_waypoints` как `Marker.SPHERE_LIST`;
- `search_area` как прямоугольник по min/max координатам waypoint-ов;
- `home` как цилиндр в точке takeoff waypoint.

Этот маршрут не читается из `mission/search_route`.

В OpenCV через `TopDownMapViewer` отображается динамический маршрут из topic-а:

```text
mission/search_route
```

Тип сообщения:

```text
std_msgs/Float32MultiArray
```

Формат данных:

```text
[x0, y0, x1, y1, x2, y2, ...]
```

Viewer декодирует массив парами, каждую точку переводит из mission-local XY в
map XY через `_mission_to_map_xy()`, затем рисует зеленую полилинию. Начало,
конец и часть промежуточных точек выделяются кругами. Подпись маршрута:

```text
search route
```

## Отображение A*-маршрута

A*-маршрут визуализируется только в `TopDownMapViewer`.

OpenCV-viewer подписывается на:

```text
mission/evacuation_route
```

Формат такой же, как у `mission/search_route`:

```text
[x0, y0, x1, y1, x2, y2, ...]
```

Viewer переводит mission-local точки в map XY, затем рисует маршрут толстой
оранжево-желтой полилинией. Все точки маршрута выделяются кругами, начало
маршрута получает подпись:

```text
A* route
```

В `SceneMapPublisher` отдельной подписки на `mission/evacuation_route` нет,
поэтому A*-маршрут в RViz-карте текущей реализацией не отображается.

## Как SDF используется для A* и покрытия зоны

В `path_planner.py` SDF-файл используется для построения двумерной карты
занятости.

### Загрузка препятствий

`EvacuationRoutePlanner` при создании вызывает `_load_obstacles()`.

Алгоритм:

1. Открыть `PlannerConfig.world_sdf_path`.
2. Прочитать XML и исправить известный формат комментариев.
3. Найти `<world>`.
4. Пройти по всем `<include>`.
5. Извлечь `uri`, `name`, имя модели из `model://...`.
6. Исключить людей и `grave`.
7. Найти footprint в `MODEL_FOOTPRINTS`.
8. Если footprint не найден, модель пропускается.
9. Построить расширенный повернутый прямоугольник.
10. Сохранить его как `Obstacle`.

### A*

При планировании маршрута `plan(start, goal)`:

1. Вычисляются границы карты по start, goal и углам препятствий.
2. Границы расширяются на `map_padding`.
3. Строится сетка с разрешением `grid_resolution`.
4. Каждая клетка помечается занятой, если ее центр лежит внутри любого
   polygon-препятствия.
5. Start и goal переводятся в клетки.
6. Если start или goal попали в занятое место, ищется ближайшая свободная
   клетка.
7. Запускается 8-связный A*.
8. Используется евклидова эвристика.
9. Найденный путь переводится обратно в XY-точки.
10. Путь сглаживается через line-of-sight проверку.
11. Waypoint-ы прореживаются по `waypoint_spacing`.

### Покрытие зоны поиска

`SearchCoveragePlanner` наследуется от `EvacuationRoutePlanner` и использует
те же препятствия.

Покрытие строится как lawnmower/snake-маршрут:

1. По препятствиям вычисляются границы покрытия.
2. Границы расширяются на `map_padding`.
3. Создаются горизонтальные строки с шагом `coverage_row_spacing`.
4. На каждой строке ищутся свободные интервалы, не попадающие внутрь
   препятствий.
5. Для каждого свободного интервала создается отрезок движения.
6. Направление отрезков чередуется: слева-направо, затем справа-налево.
7. Между несоседними отрезками строится соединение через A*.
8. Если A* не сработал, допускается прямой сегмент только при проверке
   свободной линии.
9. Из нескольких кандидатов выбирается маршрут, у которого первый переход
   ближе к старту, а затем меньше общая длина.
10. Итоговые точки прореживаются по `waypoint_spacing`.

Если препятствия не загружены, `SearchCoveragePlanner.plan_coverage()` сразу
возвращает пустой маршрут.

## Отображение дрона

Обе визуализации получают положение БПЛА из PX4 topic-а:

```text
/fmu/out/vehicle_local_position_v1
```

Тип сообщения:

```text
px4_msgs/msg/VehicleLocalPosition
```

QoS для подписки:

```text
reliability = BEST_EFFORT
durability = TRANSIENT_LOCAL
history = KEEP_LAST
depth = 1
```

Это соответствует типичному режиму работы с PX4 telemetry topic-ами в ROS 2.

### Дрон в RViz

`SceneMapPublisher` хранит последний `VehicleLocalPosition`. Если поле
`heading` конечно (`math.isfinite(msg.heading)`), оно сохраняется как текущий
heading.

Положение дрона переводится из mission-local XY в map XY и отображается как
`Marker.ARROW`.

Параметр масштаба:

```text
drone_scale = 1.2
```

Размер стрелки:

```text
scale.x = 1.8 * drone_scale
scale.y = 0.7 * drone_scale
scale.z = 0.5 * drone_scale
```

Heading для карты вычисляется как:

```text
map_heading = current_heading + radians(mission_rotation_deg)
```

### Дрон в OpenCV-viewer

`TopDownMapViewer` рисует дрон только если:

```text
vehicle_local_position is not None
vehicle_local_position.xy_valid == True
```

Положение переводится из mission-local XY в map XY, затем в пиксели.
Отображение:

- синий круг в текущей позиции;
- темная линия в направлении heading;
- текстовая подпись `drone`.

Heading переводится не простым добавлением угла, а через
`_mission_heading_to_map_heading()`: берется вектор направления в mission-local
системе, обе точки переводятся в map-систему, затем считается `atan2()`.

## Trail и точки обнаружения

Trail и detection-точки есть только в `SceneMapPublisher`.

### Trail

При каждом новом `VehicleLocalPosition` узел добавляет точку в
`trail_points`. Новая точка добавляется только если расстояние до предыдущей
точки не меньше 0.1 м. Это уменьшает количество почти одинаковых точек.

Максимальная длина trail задается параметром:

```text
trail_max_points = 500
```

Trail публикуется как `Marker.LINE_STRIP` в namespace:

```text
drone_trail
```

Ширина линии:

```text
trail_line_width = 0.12
```

### Detection-точки

`SceneMapPublisher` подписывается на:

```text
yolo/target_detected
```

Тип:

```text
std_msgs/Bool
```

Точка detection записывается только на rising edge: когда предыдущее значение
было `False`, а новое стало `True`. При этом должны выполняться условия:

- `record_detection_points=True`;
- уже получена позиция БПЛА.

Важно: detection-точка - это не координата найденного человека. Это координата
дрона в момент появления детекции. В текущем коде нет вычисления мировой
координаты человека по bbox.

Detection-точки публикуются как `Marker.SPHERE_LIST` в namespace:

```text
detections
```

## Механика `mission/person_rescued` в SceneMapPublisher

`SceneMapPublisher` подписывается на:

```text
mission/person_rescued
```

Тип:

```text
std_msgs/Bool
```

При получении `True` запускается `_person_rescued_callback()`.

Алгоритм:

1. Если `msg.data == False`, событие игнорируется.
2. Выбирается ближайший активный человек через `_nearest_active_person()`.
3. Активными считаются люди, чьи имена еще не находятся в
   `rescued_person_names`.
4. Если позиция дрона неизвестна, выбирается первый активный человек.
5. Если позиция дрона известна, она переводится из mission-local XY в map XY.
6. Выбирается человек с минимальным евклидовым расстоянием до позиции дрона.
7. Если `teleport_rescued_people=True`, узел пытается телепортировать модель.
8. Если `hide_rescued_people=True` или телепортация успешна, имя человека
   добавляется в `rescued_person_names`.
9. Люди из `rescued_person_names` больше не отображаются в RViz.

Параметры по умолчанию:

```text
teleport_rescued_people = True
hide_rescued_people = True
rescue_service_timeout_ms = 2000
teleport_world_name = ''
rescued_person_teleport_x = 200.0
rescued_person_teleport_y = 200.0
rescued_person_teleport_z = 0.5
rescued_person_teleport_spacing = 3.0
```

Если `teleport_world_name` пустой, имя мира берется из имени SDF-файла:

```python
world_name = Path(world_sdf_path).stem
```

Телепортация выполняется командой `gz service` к сервису:

```text
/world/<world_name>/set_pose
```

Запрос задает имя модели, новую позицию и ориентацию:

```text
name: "<person_model_name>"
position { x: rescued_person_teleport_x + offset
           y: rescued_person_teleport_y
           z: rescued_person_teleport_z }
orientation { x: 0 y: 0 z: 0 w: 1 }
```

`offset` равен `rescued_person_count * rescued_person_teleport_spacing`, чтобы
несколько спасенных людей не телепортировались точно в одну точку.

Если команда `gz` не найдена, истек timeout или сервис вернул ошибку, узел
пишет предупреждение и считает телепортацию неуспешной. При этом, если
`hide_rescued_people=True`, человек все равно может быть скрыт только на карте.

## Расхождение `person_rescued` с текущей миссией

В README описано, что default mission:

- публикует событие `mission/person_rescued`;
- телепортирует ближайшую модель человека из Gazebo;
- продолжает поиск после эвакуации.

`SceneMapPublisher` действительно содержит подписку на
`mission/person_rescued` и реализует реакцию на этот topic.

Однако в текущих документах `system_architecture.md`,
`mission_state_machine.md` и `target_acquisition.md` уже зафиксировано
расхождение: в текущем `node.py` publisher для `mission/person_rescued` не
найден, а миссионный узел использует собственную телепортацию модели через
параметр `rescue_model_name` и Gazebo `set_pose`/pose-modify. Быстрая проверка
по `node.py` подтверждает, что в нем создаются publisher-ы
`mission/evacuation_route` и `mission/search_route`, но отдельный publisher
для `mission/person_rescued` не обнаружен.

Итог: `SceneMapPublisher` готов реагировать на событие `mission/person_rescued`,
но текущая миссионная логика, судя по коду и документам, это событие не
публикует. Поэтому rescue-механика карты является реализованной, но сейчас
может не участвовать в основном сценарии, если отдельный publisher события не
добавлен или событие не публикуется другим узлом.

## Ключевые ROS 2 topic-и

| Topic | Тип | Кто использует | Назначение |
|---|---|---|---|
| `scene_map/markers` | `visualization_msgs/MarkerArray` | RViz | Карта сцены и маркеры миссии |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | Оба viewer-а | Положение и heading дрона |
| `yolo/target_detected` | `std_msgs/Bool` | `SceneMapPublisher` | Запись detection-точек |
| `mission/person_rescued` | `std_msgs/Bool` | `SceneMapPublisher` | Скрытие/телепортация ближайшего человека |
| `mission/search_route` | `std_msgs/Float32MultiArray` | `TopDownMapViewer` | Динамический маршрут поиска |
| `mission/evacuation_route` | `std_msgs/Float32MultiArray` | `TopDownMapViewer` | A*-маршрут возврата/эвакуации |

## Ключевые параметры

### Общие параметры SDF и кадров

| Параметр | Узел | Значение по умолчанию | Смысл |
|---|---|---|---|
| `world_sdf_path` | `SceneMapPublisher` | `.../worlds/forest.sdf` | SDF-файл для RViz-карты |
| `world_sdf_path` | `TopDownMapViewer` | `.../worlds/forest_big.sdf` | SDF-файл для OpenCV-карты |
| `parent_frame_id` | `SceneMapPublisher` | `map` | Родительский frame для RViz |
| `frame_id` | `SceneMapPublisher` | `mission_map` | Frame маркеров |
| `publish_period_sec` | `SceneMapPublisher` | `0.5` | Период публикации MarkerArray |
| `refresh_period_sec` | `TopDownMapViewer` | `0.1` | Период перерисовки OpenCV-окна |

### Параметры координат

| Параметр | Узел | Значение по умолчанию |
|---|---|---:|
| `world_swap_xy` | Оба viewer-а | `False` |
| `world_invert_x` | Оба viewer-а | `False` |
| `world_invert_y` | Оба viewer-а | `False` |
| `world_rotation_deg` | Оба viewer-а | `0.0` |
| `world_offset_x` | Оба viewer-а | `0.0` |
| `world_offset_y` | Оба viewer-а | `0.0` |
| `mission_rotation_deg` | Оба viewer-а | `90.0` |
| `mission_invert_y` | `TopDownMapViewer` | `True` |
| `mission_swap_xy` | `TopDownMapViewer` | `False` |
| `mission_invert_x` | `TopDownMapViewer` | `False` |
| `mission_offset_x` | Оба viewer-а | `0.0` |
| `mission_offset_y` | Оба viewer-а | `0.0` |

### Параметры OpenCV-viewer

| Параметр | Значение по умолчанию | Смысл |
|---|---:|---|
| `window_width` | `1100` | Ширина окна |
| `window_height` | `850` | Высота окна |
| `map_padding_m` | `5.0` | Отступ вокруг объектов |
| `object_fallback_size_m` | `2.5` | Размер объекта, если geometry не найдена |
| `min_map_size_m` | `20.0` | Минимальный размер карты в метрах |
| `draw_object_labels` | `True` | Подписи объектов |
| `draw_axes` | `True` | Оси X/Y |
| `draw_search_route` | `True` | Рисовать `mission/search_route` |
| `draw_evacuation_route` | `True` | Рисовать `mission/evacuation_route` |

### Параметры планировщика

| Параметр | Значение по умолчанию | Смысл |
|---|---:|---|
| `grid_resolution` | `1.0` | Размер клетки A* |
| `obstacle_margin` | `3.0` | Запас вокруг препятствий |
| `map_padding` | `8.0` | Расширение границ A* |
| `waypoint_spacing` | `3.0` | Минимальная дистанция между waypoint-ами |
| `coverage_row_spacing` | `7.0` | Шаг строк покрытия |
| `mission_rotation_deg` | `90.0` | Поворот между mission/map |
| `mission_invert_y` | `True` | Инверсия Y при обратном преобразовании |
| `max_iterations` | `20000` | Лимит итераций A* |

## Ограничения текущей реализации

1. SDF-парсинг читает только `<include>` внутри `<world>`. Объекты, заданные
   иначе, текущей картой не учитываются.

2. Визуализация не является полноценным физическим рендером Gazebo. Она
   строит условную двумерную карту по pose, именам моделей и приближенным
   footprint-ам.

3. `SceneMapPublisher` выбирает объекты через ключевые слова. Если имя или URI
   модели не содержит нужного токена, объект не попадет в RViz-карту.

4. `TopDownMapViewer` исключает людей и не показывает их на карте. Поэтому
   OpenCV-карта удобна для препятствий и маршрутов, но не для отображения
   текущего списка пострадавших.

5. `path_planner.py` учитывает только модели из `MODEL_FOOTPRINTS`. Новые
   модели не станут препятствиями, пока для них не добавлен footprint.

6. Размеры для RViz заданы вручную параметрами, а размеры для OpenCV частично
   извлекаются из `model.sdf` и mesh-метаданных. Поэтому визуальные размеры
   одного и того же объекта могут отличаться между RViz, OpenCV и A*.

7. `path_planner.py` не читает geometry из `model.sdf`; он не знает реальную
   форму mesh-модели. Используется прямоугольный footprint с запасом.

8. Координатные преобразования в RViz-viewer, OpenCV-viewer и планировщике
   настроены похожим образом, но не полностью одинаковы. Особенно важно, что
   OpenCV-viewer имеет `mission_invert_y=True`, а `SceneMapPublisher` не имеет
   такого параметра для mission-преобразования.

9. `SceneMapPublisher` по умолчанию использует `forest.sdf`, а
   `TopDownMapViewer` и документы по маршрутам часто используют
   `forest_big.sdf`. При разных world-файлах карта и маршруты могут не
   совпасть.

10. Detection-точки в RViz - это позиции дрона в момент появления детекции,
    а не вычисленные координаты человека.

11. A*-маршрут отображается в OpenCV-viewer, но не отображается в RViz-карте
    текущей реализацией.

12. `mission/person_rescued` реализован как входной topic для
    `SceneMapPublisher`, но текущая миссионная логика, по зафиксированному
    расхождению в документах и проверке `node.py`, не публикует этот topic.

## Формулировка для диплома

В разработанной системе SDF-файл Gazebo используется как априорная карта
симуляционной среды. Из world-файла извлекаются включенные модели, их имена,
URI и pose, после чего объекты классифицируются как статические препятствия
или модели людей. Для визуального контроля реализованы два независимых
механизма отображения: публикация `visualization_msgs/MarkerArray` для RViz и
отдельный OpenCV-viewer карты сверху. RViz-визуализация отображает объекты
сцены, людей, текущую позицию БПЛА, историю траектории и точки обнаружения, а
OpenCV-viewer дополнительно отображает динамические маршруты поиска и
эвакуации, публикуемые в ROS 2 topic-ах. Для планирования маршрутов та же
SDF-карта преобразуется в набор двумерных расширенных footprint-ов
препятствий, используемых алгоритмом A* и генератором покрытия зоны поиска.
Такой подход обеспечивает согласование симуляционной среды, планирования
движения и визуального контроля миссии, при этом точность карты ограничена
приближенным описанием геометрии моделей и выбранными преобразованиями систем
координат.
