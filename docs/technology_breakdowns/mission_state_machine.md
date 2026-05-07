# Миссионный конечный автомат

## Назначение

Миссионный конечный автомат управляет полным сценарием работы
поисково-спасательного квадрокоптера в симуляции: ожиданием данных PX4,
подготовкой `Offboard`-режима, взлетом, поиском по waypoint-ам, реакцией на
подтвержденное обнаружение человека, возвратом домой, посадкой и аварийными
переходами.

Основная реализация находится в `src/offboard_takeoff/offboard_takeoff/node.py`.
Вспомогательные структуры и функции вынесены в:

- `src/offboard_takeoff/offboard_takeoff/mission.py` - `Waypoint`,
  `MissionPlan`, статический fallback-маршрут поиска;
- `src/offboard_takeoff/offboard_takeoff/navigation.py` - сглаживание
  setpoint-ов, проверка достижения позиции и yaw;
- `src/offboard_takeoff/offboard_takeoff/path_planner.py` - построение
  автоматического маршрута поиска и A* маршрута возврата.

Сам конечный автомат задан перечислением `MissionState`:

```python
class MissionState(Enum):
    INIT = 'INIT'
    WAIT_FOR_PX4 = 'WAIT_FOR_PX4'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    HOLD = 'HOLD'
    FOLLOW_PERSON = 'FOLLOW_PERSON'
    RETURN_HOME = 'RETURN_HOME'
    LAND = 'LAND'
    FAILSAFE = 'FAILSAFE'
    FINISHED = 'FINISHED'
```

## Важное уточнение о текущей версии

README описывает публикацию события `mission/person_rescued`. В текущем
`node.py` такого publisher-а нет: миссионный узел сам вызывает `gz service`
или `gz topic` и телепортирует модель, указанную параметром
`rescue_model_name`. При этом `scene_map_publisher.py` действительно
подписывается на `mission/person_rescued`, поэтому документация README и
текущая реализация расходятся. В этом документе ниже описана именно текущая
логика `node.py`.

Также README упоминает параметр `resume_search_after_rescue`; в текущем
`node.py` он не объявлен. Продолжение поиска после возврата домой реализовано
внутренними флагами `pending_search_resume_after_home`,
`pending_teleport_after_home` и индексом `resume_search_waypoint_index`.

## Общий цикл работы

Узел `OffboardTakeoff` запускает таймер с частотой:

```text
control_rate_hz = 10.0
```

По умолчанию основной цикл `control_loop()` вызывается примерно каждые
0,1 секунды. В каждом цикле выполняются следующие действия:

1. если состояние равно `FINISHED`, управление сразу завершается;
2. вызывается централизованная проверка безопасности `check_safety()`;
3. периодически повторно публикуется маршрут поиска для viewer-ов;
4. выбирается обработчик текущего состояния;
5. если состояние требует удержания `Offboard`, публикуются
   `OffboardControlMode` и `TrajectorySetpoint`.

Setpoint-ы публикуются в состояниях:

```text
WAIT_FOR_PX4
ARMING
TAKEOFF
SEARCH
HOLD
FOLLOW_PERSON
RETURN_HOME
FAILSAFE
```

В `INIT`, `LAND` и `FINISHED` обычные offboard setpoint-ы не публикуются.
В `LAND` вместо этого отправляется команда посадки PX4.

## ROS 2 topic-и

### Входные topic-и

| Topic | Тип | Назначение |
| --- | --- | --- |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | Положение, высота, курс и флаги валидности локальной позиции PX4. |
| `/fmu/out/vehicle_status_v2` | `px4_msgs/VehicleStatus` | Состояние PX4: arm/disarm, nav state, failsafe. |
| `yolo/target_bbox` | `std_msgs/Int32MultiArray` | Лучший bbox человека от YOLO-детектора. |

Сообщение `yolo/target_bbox` ожидается в формате:

```text
[confidence_milli, center_x, center_y, width, height, image_width, image_height]
```

Если длина массива меньше 7, текущая цель сбрасывается.

### Выходные topic-и

| Topic | Тип | Назначение |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Сообщает PX4, что внешний контроллер управляет позицией. |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Целевая позиция `[x, y, z]` и yaw в локальной системе PX4. |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Команды `ARM`, переход в `OFFBOARD`, `LAND`. |
| `mission/search_route` | `std_msgs/Float32MultiArray` | Список точек маршрута поиска для отображения на карте. |
| `mission/evacuation_route` | `std_msgs/Float32MultiArray` | Список точек A* маршрута возврата для отображения на карте. |

Формат `mission/search_route` и `mission/evacuation_route` одинаковый:
последовательность чисел `[x0, y0, x1, y1, ...]`.

## Основные параметры

| Параметр | Значение по умолчанию | Назначение |
| --- | ---: | --- |
| `control_rate_hz` | `10.0` | Частота основного цикла управления. |
| `takeoff_height` | `7.0` | Рабочая высота в метрах; в NED используется как отрицательное `z`. |
| `max_speed` | `3.0` | Максимальная скорость изменения позиционного setpoint-а. |
| `max_yaw_rate` | `0.5` | Максимальная скорость изменения yaw. |
| `waypoint_tolerance` | `0.4` | Радиус достижения waypoint-а. |
| `yaw_tolerance` | `0.3` | Допуск по yaw для взлета и поиска. |
| `required_initial_setpoints` | `30` | Количество начальных setpoint-ов перед запросом `Offboard`. |
| `arming_timeout_sec` | `5.0` | Таймаут fallback-перехода из `ARMING`, если нет статуса PX4. |
| `waypoint_timeout_sec` | `120.0` | Таймаут достижения активного waypoint-а. |
| `return_home_on_failure` | `True` | Разрешает попытку возврата домой при ошибке. |
| `auto_land_on_finish` | `True` | После окончания поиска возвращаться домой и садиться. |
| `enable_person_follow` | `True` | Разрешает переход к визуальному подлету к человеку. |
| `enable_vision` | `False` | Включает общий timeout контроля поступления vision-данных в `SEARCH/HOLD`. |
| `enable_auto_search_route` | `True` | Пытаться строить маршрут поиска по SDF-карте. |
| `enable_evacuation_astar` | `True` | Пытаться строить A* маршрут возврата после взаимодействия с человеком. |

## Waypoint-ы и target setpoint

В проекте используется структура `Waypoint`:

```python
@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float
    hold_time: float = 0.0
```

`MissionPlan` содержит:

- `takeoff_waypoint` - точка взлета на рабочей высоте;
- `search_waypoints` - последовательность точек маршрута поиска.

В `node.py` активный waypoint зависит от состояния:

| Состояние | Активный waypoint |
| --- | --- |
| `TAKEOFF` | `mission_plan.takeoff_waypoint` |
| `SEARCH`, `HOLD` | `current_search_waypoint` |
| `RETURN_HOME` | `return_home_waypoint` |
| остальные | активный waypoint отсутствует |

Командуемый PX4 setpoint хранится отдельно как `TargetState`. Он не
перескакивает мгновенно на waypoint, а плавно сдвигается функцией
`smooth_target_towards_waypoint()` с ограничением по `max_speed` и
`max_yaw_rate`. Достижение waypoint-а проверяется по расстоянию, а для
`TAKEOFF`, `SEARCH` и `HOLD` дополнительно по yaw.

## Построение маршрута поиска

При старте узел вызывает `_build_mission_plan()`.

Если `enable_auto_search_route=True`, используется `SearchCoveragePlanner` из
`path_planner.py`. Он получает параметры SDF-карты, сетки, отступов от
препятствий и преобразования координат:

```text
search_world_sdf_path
search_grid_resolution
search_obstacle_margin
search_map_padding
search_row_spacing
search_waypoint_spacing
search_mission_rotation_deg
search_mission_invert_y
search_max_iterations
```

Если автоматический маршрут не построен или содержит меньше двух точек,
используется fallback из `mission.py`: статический маршрут змейкой над
лесной областью. В fallback рабочая высота равна `-abs(takeoff_height)`, а
некоторые точки имеют `hold_time=1.5`.

Автоматический маршрут преобразуется в `Waypoint`-ы функцией
`_waypoints_from_search_route()`. Для каждой точки вычисляется yaw по
направлению движения. `hold_time` добавляется в последней точке и в точках,
где направление маршрута заметно меняется.

## Таблица состояний

| Состояние | Задача | Основной обработчик | Основные выходы |
| --- | --- | --- | --- |
| `INIT` | Дождаться валидной локальной позиции и сохранить home. | `handle_init()` | Нет offboard setpoint-ов. |
| `WAIT_FOR_PX4` | Отправить предварительные setpoint-ы перед `Offboard`. | `handle_wait_for_px4()` | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`. |
| `ARMING` | Запросить `Offboard` и arm, дождаться подтверждения PX4. | `handle_arming()` | Setpoint-ы, команды `DO_SET_MODE` и `ARM`. |
| `TAKEOFF` | Выйти на рабочую высоту. | `handle_takeoff()` | Плавный позиционный setpoint к `takeoff_waypoint`. |
| `SEARCH` | Лететь по маршруту поиска и проверять цель. | `handle_search()` | Позиционный setpoint к текущему search waypoint. |
| `HOLD` | Удерживать waypoint с заданным `hold_time`. | `handle_hold()` | Позиционный setpoint к текущему waypoint. |
| `FOLLOW_PERSON` | Выполнить подтвержденный визуальный подлет к человеку. | `handle_follow_person()` | Setpoint удержания, поворота yaw и движения вперед. |
| `RETURN_HOME` | Вернуться домой напрямую или по A* маршруту. | `handle_return_home()` | Позиционный setpoint к `return_home_waypoint` или A* waypoint. |
| `LAND` | Отправить команду посадки и дождаться disarm. | `handle_land()` | `/fmu/in/vehicle_command` с `VEHICLE_CMD_NAV_LAND`. |
| `FAILSAFE` | Удерживать текущий target без активного восстановления. | `handle_failsafe()` | Offboard setpoint на удержание позиции. |
| `FINISHED` | Завершить миссию. | Нет активного обработчика | Нет новых миссионных действий. |

## Условия переходов

### `INIT -> WAIT_FOR_PX4`

Условие: получена валидная локальная позиция PX4:

```text
vehicle_local_position.xy_valid == True
vehicle_local_position.z_valid == True
```

При первом валидном положении узел сохраняет `home_position` из текущих
`x`, `y`, `z` и `heading`. Если `heading` не является конечным числом,
используется yaw `0.0`.

### `WAIT_FOR_PX4 -> ARMING`

Условие: отправлено `required_initial_setpoints = 30` начальных setpoint-ов.
После этого вызывается `_request_offboard_and_arm()`, то есть отправляются:

- `VEHICLE_CMD_DO_SET_MODE` с `param1=1.0`, `param2=6.0`;
- `VEHICLE_CMD_COMPONENT_ARM_DISARM` с `param1=1.0`.

### `ARMING -> TAKEOFF`

Основное условие: PX4 сообщает, что аппарат вооружен и находится в
`NAVIGATION_STATE_OFFBOARD`:

```text
arming_state == ARMING_STATE_ARMED
nav_state == NAVIGATION_STATE_OFFBOARD
```

Если сообщения `VehicleStatus` вообще не приходят и состояние длится не
меньше `arming_timeout_sec`, код допускает legacy fallback и тоже переходит в
`TAKEOFF`. Если статус приходит, но подтверждения arm/offboard нет, узел
повторяет запрос `Offboard` и `ARM` примерно раз в секунду.

### `TAKEOFF -> SEARCH`

Условие: достигнут `takeoff_waypoint` по позиции и yaw. Позиция проверяется
через `waypoint_tolerance`, yaw через `yaw_tolerance`.

Если `search_waypoints` пустой, миссия сразу завершается через
`_complete_mission()`.

В текущем коде `handle_takeoff()` также вызывает `_maybe_start_person_follow()`
пока точка взлета еще не достигнута, но сама функция разрешает запуск
follow-режима только из `SEARCH` или `HOLD`. Поэтому фактически обнаружение
человека не переводит миссию в `FOLLOW_PERSON` во время `TAKEOFF`.

### `SEARCH -> HOLD`

Условие: текущий waypoint достигнут, и у него `hold_time > 0.0`.

### `SEARCH -> SEARCH`

Условие: текущий waypoint достигнут, `hold_time == 0.0`, и есть следующий
waypoint. Узел увеличивает `current_search_waypoint_index`, сбрасывает
таймер активного waypoint-а и продолжает поиск.

### `SEARCH -> FOLLOW_PERSON`

Условие: `_maybe_start_person_follow()` подтвердил цель. Подтверждение
описано ниже в отдельном разделе.

Перед переходом сохраняется индекс, с которого потом можно возобновить поиск:

```text
resume_search_waypoint_index = current_search_waypoint_index
```

Если текущий индекс отсутствует, но маршрут поиска не пустой, используется
индекс `0`.

### `SEARCH -> RETURN_HOME/LAND/FINISHED`

Когда waypoint-ы поиска закончились, вызывается `_complete_mission()`.

Дальнейшее зависит от параметров и наличия home:

- если `auto_land_on_finish=False`, переход сразу в `FINISHED`;
- если `auto_land_on_finish=True` и `home_position` известна, переход в
  `RETURN_HOME`;
- если `auto_land_on_finish=True`, но home нет, переход в `LAND`.

### `HOLD -> SEARCH`

Условие: время в состоянии `HOLD` стало не меньше `current_search_waypoint.hold_time`.
После этого вызывается `_advance_search_or_finish()`.

### `HOLD -> FOLLOW_PERSON`

Условие такое же, как в `SEARCH -> FOLLOW_PERSON`: цель должна пройти
подтверждение в `_maybe_start_person_follow()`.

### `FOLLOW_PERSON -> RETURN_HOME`

Условие: завершена внутренняя последовательность визуального подлета:

```text
STOP_BEFORE_APPROACH
ALIGN_TO_TARGET
APPROACH_FORWARD
HOLD_AFTER_APPROACH
```

После выдержки `follow_hover_after_approach_sec` вызывается
`_finish_person_action()`. Она:

1. выставляет `pending_search_resume_after_home=True`;
2. выставляет `pending_teleport_after_home=True`;
3. готовит прямой home waypoint;
4. пытается построить A* маршрут возврата;
5. переводит автомат в `RETURN_HOME`.

### `FOLLOW_PERSON -> SEARCH`

Такой переход возможен во время `ALIGN_TO_TARGET`, если цель потеряна и ее
не удалось восстановить по памяти ошибки. Тогда `_resume_search_after_alignment_loss()`
возвращает миссию в `SEARCH`, если текущий search waypoint существует.

Если search waypoint отсутствует, вместо возврата к поиску выполняется
переход в `FAILSAFE`.

### `RETURN_HOME -> RETURN_HOME`

Если построен A* маршрут, `RETURN_HOME` последовательно проходит элементы
`evacuation_route_waypoints`. При достижении текущей точки
`_advance_evacuation_route()` выбирает следующую точку и остается в
`RETURN_HOME`.

### `RETURN_HOME -> SEARCH`

Если `pending_search_resume_after_home=True`, после достижения последней
точки возврата вызывается `_resume_search_after_home_reached()`. Она:

- удерживает текущую позицию;
- очищает сохраненную цель;
- запускает cooldown игнорирования детекций;
- при необходимости телепортирует модель `rescue_model_name`;
- выбирает `resume_search_waypoint_index`;
- переводит автомат обратно в `SEARCH`.

### `RETURN_HOME -> LAND`

Если возврат домой завершен и `pending_search_resume_after_home=False`,
автомат переходит в `LAND`.

### `LAND -> FINISHED`

В `LAND` один раз отправляется команда:

```text
VEHICLE_CMD_NAV_LAND
```

После этого узел ждет, пока `VehicleStatus` покажет, что аппарат больше не
вооружен. При `not _is_armed()` состояние меняется на `FINISHED`.

### `FAILSAFE -> FINISHED`

В `FAILSAFE` узел не отправляет команду посадки. Он удерживает последний
target и завершает миссию только если PX4 уже сообщает disarm.

## Подтверждение цели и переход в `FOLLOW_PERSON`

Переход к человеку выполняется только из `SEARCH` или `HOLD`, только если
`enable_person_follow=True` и не активен post-rescue cooldown.

Текущий bbox считается пригодным, если:

- сообщение `yolo/target_bbox` было получено и не устарело дольше
  `follow_person_timeout_sec`;
- `confidence >= follow_min_confidence`;
- ширина и высота bbox проходят пороги для текущего режима.

Для `SEARCH/HOLD` используются:

```text
search_min_bbox_width_px = 40
search_min_bbox_height_px = 40
```

Для `FOLLOW_PERSON` используются:

```text
follow_min_bbox_width_px = 35
follow_min_bbox_height_px = 35
```

Цель должна удерживаться в кадре минимум:

```text
follow_detection_confirm_sec = 1.0
```

Если bbox пропадает, но пропадание длится не дольше:

```text
follow_detection_gap_tolerance_sec = 0.5
```

подтверждение не сбрасывается. Более длинная потеря цели обнуляет окно
подтверждения.

Пока подтверждение еще идет, узел вызывает `_hold_current_position_target()`,
то есть фиксирует текущую позицию как target. Это останавливает движение по
маршруту, пока система проверяет, что обнаружение не случайное.

После успешного подтверждения:

- активный waypoint timeout перезапускается;
- запоминается индекс waypoint-а для последующего возобновления поиска;
- состояние меняется на `FOLLOW_PERSON`;
- `person_reference_bbox_height` получает текущую высоту bbox.

## Внутренние фазы `FOLLOW_PERSON`

### `STOP_BEFORE_APPROACH`

Дрон удерживает текущую позицию через `_hold_current_position_target()`.
Длительность фазы:

```text
follow_stop_before_approach_sec = 1.0
```

После выдержки начинается `ALIGN_TO_TARGET`.

### `ALIGN_TO_TARGET`

Цель - повернуть yaw так, чтобы горизонтальная ошибка bbox стала достаточно
малой. Ошибка вычисляется как:

```text
center_x_error = (center_x - image_width / 2) / (image_width / 2)
```

Если `abs(center_x_error) <= follow_yaw_deadband`, дрон считает цель
выровненной и запускает `APPROACH_FORWARD`.

Если свежий bbox временно недоступен, код пытается использовать память:
`last_stable_person_target` или `last_significant_person_center_x_error`.
Если память не дает пригодную ошибку или цель не восстановлена дольше
`follow_align_dropout_continue_sec`, автомат возвращается к поиску.

Yaw изменяется ограниченно:

```text
yaw_rate_cmd = follow_yaw_kp * yaw_error
yaw_rate_cmd ограничивается диапазоном [-max_yaw_rate, max_yaw_rate]
```

### `APPROACH_FORWARD`

Дрон движется вперед по текущему heading с небольшой скоростью:

```text
follow_forward_speed = 0.35
```

Дальность до человека напрямую не измеряется. Вместо этого используется рост
высоты bbox относительно высоты, запомненной в момент подтверждения:

```text
current_bbox_height >= person_reference_bbox_height * follow_bbox_growth_target
```

По умолчанию:

```text
follow_bbox_growth_target = 4.0
```

Подлет также завершается, если:

- превышен `follow_approach_max_sec = 12.0`;
- bbox подошел к нижнему краю кадра с учетом `follow_bottom_edge_margin_ratio`;
- цель потеряна дольше `follow_approach_dropout_stop_sec = 0.5`.

После завершения forward-подлета автомат переходит во внутреннюю фазу
`HOLD_AFTER_APPROACH`.

### `HOLD_AFTER_APPROACH`

Дрон удерживает позицию рядом с целью:

```text
follow_hover_after_approach_sec = 5.0
```

Затем вызывается `_finish_person_action()`, и миссия переходит в
`RETURN_HOME`.

### Общий timeout `FOLLOW_PERSON`

Вся реакция на человека ограничена:

```text
follow_person_action_timeout_sec = 90.0
```

Если timeout превышен, `check_safety()` вызывает `handle_failure()`.

## Возврат домой и A* маршрут

Прямой home waypoint создается функцией `_prepare_return_home_waypoint()`.
Цель возврата:

- `x = home_position.x`;
- `y = home_position.y`;
- `z = mission_plan.takeoff_waypoint.z`, то есть возврат идет на рабочей
  высоте, а не на исходной высоте home;
- `yaw = target.yaw`, если текущий target существует, иначе `home_position.yaw`.

После завершения действия с человеком дополнительно вызывается
`_prepare_evacuation_route()`. A* строится только если:

- `enable_evacuation_astar=True`;
- есть текущая локальная позиция;
- есть `home_position`;
- уже подготовлен `return_home_waypoint`.

Параметры A* возврата:

```text
evacuation_world_sdf_path
evacuation_grid_resolution = 1.0
evacuation_obstacle_margin = 3.0
evacuation_map_padding = 8.0
evacuation_waypoint_spacing = 3.0
evacuation_mission_rotation_deg = 90.0
evacuation_mission_invert_y = True
evacuation_max_iterations = 20000
```

Планировщик получает старт из текущих `vehicle_local_position.x/y`, а цель
из `return_home_waypoint.x/y`. Если маршрут найден и содержит минимум две
точки, первая точка отбрасывается как текущая позиция дрона, остальные точки
становятся `evacuation_route_waypoints`. Первый A* waypoint назначается
активным `return_home_waypoint`.

Если A* маршрут не найден, код явно пишет предупреждение и оставляет прямой
возврат домой.

Маршрут A* публикуется в `mission/evacuation_route` только при успешном
построении.

## Телепортация модели после спасения

В текущей реализации телепортация выполняется не в момент обнаружения и не
сразу после подлета. После `FOLLOW_PERSON` дрон возвращается домой. Только
после достижения home и перед возобновлением поиска вызывается
`_resume_search_after_home_reached()`, где при активном
`pending_teleport_after_home` вызывается `_teleport_rescue_model_down()`.

Телепортируется одна заранее заданная модель:

```text
rescue_model_name = man_3
rescue_model_teleport_x = 36.57
rescue_model_teleport_y = 28.2
rescue_model_teleport_z = -10.0
teleport_world_name = forest_big
```

Код сначала пробует `gz service -s /world/<world>/set_pose`, затем fallback
через `gz topic -t /world/<world>/pose/modify`. Если указанное имя мира не
сработало, перебираются кандидаты:

```text
forest_big
forest
default
empty
```

После возврата домой запускается cooldown:

```text
post_rescue_detection_cooldown_sec = 5.0
```

Во время cooldown новые bbox игнорируются, чтобы дрон не захватил ту же цель
сразу после возобновления маршрута.

## Проверки безопасности

Централизованная функция `check_safety()` вызывается в начале каждого
`control_loop()`. Проверки не применяются к состояниям:

```text
INIT
LAND
FAILSAFE
FINISHED
```

В остальных состояниях проверяется:

| Проверка | Условие ошибки | Реакция |
| --- | --- | --- |
| Нет local position после старта | `last_local_position_received_at is None` дольше `px4_data_timeout_sec` | `handle_failure()` |
| Устарела local position | возраст данных больше `px4_data_timeout_sec` | `handle_failure()` |
| Устарел VehicleStatus | если статус уже был и устарел больше `px4_data_timeout_sec` | `handle_failure()` |
| PX4 failsafe | `vehicle_status.failsafe == True` | `handle_failure()` |
| Waypoint timeout | `TAKEOFF`, `SEARCH`, `RETURN_HOME` идут дольше `waypoint_timeout_sec` | `handle_failure()` |
| Follow timeout | `FOLLOW_PERSON` идет дольше `follow_person_action_timeout_sec` | `handle_failure()` |
| Hold timeout | `HOLD` длится дольше `hold_time + hold_timeout_margin_sec` | `handle_failure()` |
| Vision timeout | при `enable_vision=True` данные vision устарели в `SEARCH/HOLD` | `handle_failure()` |

Поведение при ошибке задает `handle_failure()`:

1. сохраняется `failure_reason`;
2. если `_can_return_home_after_failure()` возвращает `True`, готовится
   home waypoint и состояние меняется на `RETURN_HOME`;
3. иначе target фиксируется в текущей позиции, и состояние меняется на
   `FAILSAFE`.

Возврат домой после ошибки разрешен только если:

- `return_home_on_failure=True`;
- `home_position` известна;
- локальная позиция валидна;
- local position и vehicle status не устарели;
- PX4 сообщает, что аппарат вооружен.

## Упрощенная схема переходов

```text
INIT
  -> WAIT_FOR_PX4
  -> ARMING
  -> TAKEOFF
  -> SEARCH
       -> SEARCH
       -> HOLD -> SEARCH
       -> FOLLOW_PERSON
             -> SEARCH        при потере цели во время ALIGN_TO_TARGET
             -> RETURN_HOME   после завершения визуального подлета
       -> RETURN_HOME         после окончания маршрута поиска
  -> RETURN_HOME
       -> RETURN_HOME         переход между A* waypoint-ами
       -> SEARCH              после возврата с эвакуацией и cooldown
       -> LAND                после финального возврата домой
  -> LAND
  -> FINISHED

При ошибке безопасности:
  активное состояние -> RETURN_HOME, если возврат безопасен
  активное состояние -> FAILSAFE, если возврат невозможен

FAILSAFE -> FINISHED после disarm
LAND -> FINISHED после disarm
```

## Ограничения текущей реализации

- Реакция на человека основана на bbox из монокулярного изображения. Реальная
  дальность до человека не измеряется; приближение оценивается по росту bbox.
- Телепортация спасенного человека является симуляционным упрощением, а не
  физической моделью эвакуации.
- В текущем `node.py` телепортируется конкретная модель `rescue_model_name`,
  а не автоматически выбранная ближайшая модель человека.
- `mission/person_rescued` описан в README и используется
  `scene_map_publisher.py`, но миссионный узел сейчас это событие не
  публикует.
- Переход к `FOLLOW_PERSON` фактически разрешен только из `SEARCH` и `HOLD`;
  обнаружение во время `TAKEOFF` не запускает подлет.
- В `FAILSAFE` узел удерживает target и ждет disarm, но не отправляет
  отдельную команду посадки.
- `enable_vision=False` по умолчанию, поэтому общий vision timeout в
  `SEARCH/HOLD` выключен, хотя bbox для `FOLLOW_PERSON` все равно проверяется
  на свежесть через `follow_person_timeout_sec`.

## Формулировка для диплома

Логика автономной миссии реализована в виде конечного автомата, встроенного в
ROS 2 узел управления PX4 Offboard. Каждое состояние автомата соответствует
отдельному этапу полета: инициализации и сохранению домашней позиции,
подготовке Offboard-режима, взлету, поиску по маршруту покрытия, удержанию в
точках маршрута, визуальному подлету к обнаруженному человеку, возврату домой,
посадке и аварийному режиму. Такой подход позволяет явно задать условия
переходов, отделить штатные этапы миссии от реакций на ошибки и централизовать
проверки безопасности.

Маршрут поиска представлен последовательностью waypoint-ов в локальной системе
PX4. Командуемый setpoint не изменяется скачком, а плавно движется к активному
waypoint-у с ограничением скорости и скорости поворота, что уменьшает резкие
изменения управляющих команд. При подтвержденном обнаружении человека миссия
временно прерывает поиск, удерживает позицию, выравнивает yaw по положению
bounding box в кадре, выполняет подлет вперед до увеличения bbox и затем
возвращается домой. Для возврата после взаимодействия с человеком может быть
построен A* маршрут по карте препятствий, полученной из SDF-сцены Gazebo; если
планирование не удалось, используется прямой возврат к домашней позиции.
