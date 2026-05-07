# PX4 Offboard управление

## Назначение

PX4 Offboard управление в этом проекте используется для автономного полета
квадрокоптера из ROS 2. Логика миссии находится не внутри PX4, а во внешнем
ROS 2 узле `offboard_takeoff`. Этот узел получает состояние аппарата из PX4,
вычисляет текущую целевую точку и регулярно отправляет в PX4 управляющие
сообщения.

В рамках проекта Offboard-контур отвечает за:

- ожидание готовности PX4 и получение локальной позиции;
- сохранение домашней точки;
- перевод аппарата в Offboard mode;
- arm-команду;
- взлет на рабочую высоту;
- движение по поисковым waypoint-ам;
- удержание точки;
- визуальное наведение на найденного человека;
- возврат домой;
- посадку;
- обработку аварийных ситуаций.

Основной файл реализации:

```text
src/offboard_takeoff/offboard_takeoff/node.py
```

Вспомогательные структуры и функции:

```text
src/offboard_takeoff/offboard_takeoff/mission.py
src/offboard_takeoff/offboard_takeoff/navigation.py
```

Точка входа ROS 2 executable:

```text
src/offboard_takeoff/offboard_takeoff/offboard_takeoff.py
```

Executable зарегистрирован в:

```text
src/offboard_takeoff/setup.py
```

как:

```text
offboard_takeoff = offboard_takeoff.offboard_takeoff:main
```

## Что такое Offboard в контексте проекта

В коде проекта Offboard означает режим, в котором PX4 получает команды
движения от внешнего ROS 2 узла. Узел `OffboardTakeoff` публикует не ручные
команды моторам, а высокоуровневые setpoint-ы положения и yaw. PX4 остается
автопилотом, а ROS 2 узел задает желаемую траекторию.

Контур работает по таймеру:

```text
control_rate_hz = 10.0
```

По умолчанию основной цикл вызывается примерно 10 раз в секунду. В каждом
цикле `control_loop()`:

1. выполняет централизованные проверки безопасности;
2. вызывает обработчик текущего состояния миссии;
3. при необходимости публикует Offboard setpoint-ы в PX4.

Состояния, в которых публикуются Offboard setpoint-ы:

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

В состояниях `INIT`, `LAND` и `FINISHED` регулярные Offboard setpoint-ы не
публикуются. В `LAND` узел отправляет отдельную PX4-команду посадки.

## Используемые PX4 topic-и

Узел публикует три PX4 topic-а:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Узел подписывается на два PX4 topic-а:

```text
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v2
```

Подписки на PX4 output topic-и создаются с QoS:

```text
ReliabilityPolicy.BEST_EFFORT
DurabilityPolicy.TRANSIENT_LOCAL
HistoryPolicy.KEEP_LAST
depth = 1
```

Это видно в `node.py` при создании `px4_qos`.

Отдельно от PX4 узел также подписывается на:

```text
yolo/target_bbox
```

Этот topic не является PX4 topic-ом. Он используется для связи детектора
человека с миссионной логикой.

## `/fmu/in/offboard_control_mode`

Сообщение `OffboardControlMode` публикуется функцией:

```python
publish_offboard_control_mode()
```

В проекте включен только позиционный режим:

```text
position = True
velocity = False
acceleration = False
attitude = False
body_rate = False
```

Это означает, что миссия отправляет в PX4 целевое положение, а не целевую
скорость, ускорение, ориентацию или body-rate. Управление скоростью полета
реализовано на стороне ROS 2 узла через постепенное изменение позиции setpoint-а.

Каждое сообщение получает timestamp в микросекундах:

```python
msg.timestamp = self.timestamp_us()
```

Функция `timestamp_us()` берет текущее ROS-время и переводит наносекунды в
микросекунды:

```python
self.get_clock().now().nanoseconds // 1000
```

## `/fmu/in/trajectory_setpoint`

Сообщение `TrajectorySetpoint` публикуется функцией:

```python
publish_trajectory_setpoint()
```

В него записываются:

```text
position = [target.x, target.y, target.z]
yaw = target.yaw
```

В коде не задаются поля velocity, acceleration или jerk. Основная команда
движения для PX4 в этом проекте - это текущая сглаженная целевая позиция и
угол yaw.

Текущее целевое состояние хранится в структуре `TargetState` из
`navigation.py`:

```python
@dataclass
class TargetState:
    x: float
    y: float
    z: float
    yaw: float
```

## `/fmu/in/vehicle_command`

Команды PX4 публикуются функцией:

```python
publish_vehicle_command()
```

Для всех команд в проекте задаются:

```text
target_system = 1
target_component = 1
source_system = 1
source_component = 1
from_external = True
```

Используются следующие команды:

```text
VEHICLE_CMD_DO_SET_MODE
VEHICLE_CMD_COMPONENT_ARM_DISARM
VEHICLE_CMD_NAV_LAND
```

## Начальные setpoint-ы перед Offboard

После получения валидной локальной позиции узел переходит из `INIT` в
`WAIT_FOR_PX4`. В этом состоянии он еще не отправляет arm-команду и еще не
запрашивает Offboard mode. Сначала он публикует начальные setpoint-ы на точку
взлета.

Количество начальных setpoint-ов задано в конструкторе:

```text
required_initial_setpoints = 30
```

При стандартной частоте:

```text
control_rate_hz = 10.0
```

это соответствует примерно 3 секундам публикации. В коде это реализовано через
счетчик `initial_setpoint_counter` в `handle_wait_for_px4()`.

Текущий target в это время устанавливается в точку взлета:

```python
self.target = self._target_from_waypoint(
    self.mission_plan.takeoff_waypoint
)
```

После отправки 30 начальных setpoint-ов вызывается:

```python
self._request_offboard_and_arm()
```

и состояние меняется:

```text
WAIT_FOR_PX4 -> ARMING
```

В документации `mission_state_machine.md` прямо указано, что PX4 требует
предварительной публикации setpoint-ов перед включением Offboard mode. В самом
коде это также закреплено в названии состояния и docstring-е:

```text
Publish initial setpoints before requesting offboard mode.
```

## Перевод в Offboard mode и arm

Перевод в Offboard и arm объединены в функции:

```python
_request_offboard_and_arm()
```

Она последовательно вызывает:

```python
self.engage_offboard_mode()
self.arm()
```

### Команда Offboard mode

Функция:

```python
engage_offboard_mode()
```

публикует:

```text
command = VEHICLE_CMD_DO_SET_MODE
param1 = 1.0
param2 = 6.0
```

В коде эта команда логируется как:

```text
OFFBOARD mode command
```

### Команда arm

Функция:

```python
arm()
```

публикует:

```text
command = VEHICLE_CMD_COMPONENT_ARM_DISARM
param1 = 1.0
```

В коде команда логируется как:

```text
ARM command
```

### Ожидание подтверждения

В состоянии `ARMING` узел ждет, пока PX4 через `VehicleStatus` подтвердит:

```text
arming_state == ARMING_STATE_ARMED
nav_state == NAVIGATION_STATE_OFFBOARD
```

Проверки вынесены в функции:

```python
_is_armed()
_is_offboard_active()
```

Если оба условия выполнены, миссия переходит:

```text
ARMING -> TAKEOFF
```

Если статус PX4 недоступен, в коде предусмотрен fallback после таймаута
`arming_timeout_sec`. По умолчанию:

```text
arming_timeout_sec = 5.0
```

Если подтверждение еще не получено, команда Offboard+arm повторяется не чаще
одного раза в секунду:

```python
self.age_sec(self.last_offboard_request_at) >= 1.0
```

## Локальная система координат и отрицательная высота

В `mission.py` структура `Waypoint` описана как точка в PX4 local NED:

```python
"""Single waypoint in PX4 local NED coordinates."""
```

Waypoint содержит:

```text
x
y
z
yaw
hold_time
```

Рабочая высота формируется как отрицательное значение:

```python
working_altitude = -abs(takeoff_height)
```

Это используется и в статическом маршруте `default_mission_plan()`, и при
автоматической генерации маршрута поиска в `node.py`.

По умолчанию в текущем `node.py`:

```text
takeoff_height = 7.0
```

Следовательно, рабочая координата высоты:

```text
z = -7.0
```

Это важный момент для дипломного описания: параметр `takeoff_height` задается
как положительная величина высоты, но в PX4 local NED waypoint отправляется с
отрицательной координатой `z`.

Точка взлета при автоматически сгенерированном маршруте задается как:

```text
x = 0.0
y = 0.0
z = -abs(takeoff_height)
yaw = 0.0
```

В статическом fallback-маршруте используется тот же принцип.

## Сохранение home position

Домашняя точка сохраняется в состоянии `INIT`, когда появилась валидная
локальная позиция:

```python
self._has_valid_local_position()
```

Проверка валидности использует поля PX4 `VehicleLocalPosition`:

```text
xy_valid
z_valid
```

Если позиция валидна, вызывается:

```python
self.home_position = self._make_home_position()
```

Домашняя точка строится из текущих данных PX4:

```text
x = vehicle_local_position.x
y = vehicle_local_position.y
z = vehicle_local_position.z
yaw = vehicle_local_position.heading
```

Если heading не является конечным числом, используется `0.0`.

В дальнейшем home position применяется для возврата домой после завершения
миссии, после взаимодействия с найденным человеком и при части аварийных
сценариев.

## Сглаживание setpoint-ов через `navigation.py`

PX4 получает не резкие прыжки между waypoint-ами, а сглаженную целевую точку.
За это отвечает функция:

```python
smooth_target_towards_waypoint()
```

Она находится в:

```text
src/offboard_takeoff/offboard_takeoff/navigation.py
```

Функция получает:

```text
target
waypoint
control_period
max_speed
max_yaw_rate
```

и возвращает новый `TargetState`.

### Ограничение линейного шага

Сначала вычисляется максимальный шаг по позиции за один цикл:

```python
max_position_step = max_speed * control_period
```

При значениях по умолчанию:

```text
control_rate_hz = 10.0
control_period = 0.1 s
max_speed = 3.0 m/s
```

максимальный сдвиг target-а за один цикл составляет:

```text
0.3 m
```

Функция `limit_vector_step()` ограничивает трехмерный вектор движения до этой
длины, сохраняя направление. Поэтому если waypoint далеко, target постепенно
движется к нему. Если waypoint близко, target доходит до него без
перелета по команде.

### Ограничение yaw

Yaw нормализуется в диапазон:

```text
[-pi, pi]
```

через:

```python
normalize_angle()
```

Шаг yaw ограничивается:

```python
max_yaw_rate * control_period
```

При значениях по умолчанию:

```text
max_yaw_rate = 0.5 rad/s
control_period = 0.1 s
```

максимальное изменение yaw target-а за один цикл:

```text
0.05 rad
```

Это значит, что ограничение скорости разворота реализовано на уровне
генерации setpoint-а, а не через PX4 velocity/body-rate interface.

## Взлет и движение по waypoint-ам

После подтверждения arm и Offboard mode миссия переходит в `TAKEOFF`.
Активным waypoint-ом становится `mission_plan.takeoff_waypoint`.

В `TAKEOFF`, `SEARCH`, `HOLD` и `RETURN_HOME` целевая точка обновляется через:

```python
_update_smoothed_target()
```

Эта функция берет `active_waypoint` и передвигает `self.target` к нему через
`smooth_target_towards_waypoint()`.

Достижение waypoint-а проверяется функцией:

```python
_has_reached_active_waypoint(check_yaw=True или False)
```

Позиция считается достигнутой, если расстояние до waypoint-а меньше или равно:

```text
waypoint_tolerance = 0.4
```

Yaw считается достигнутым, если ошибка yaw меньше или равна:

```text
yaw_tolerance = 0.3
```

Для взлета и поиска используется проверка yaw:

```text
check_yaw = True
```

Для возврата домой yaw не проверяется:

```text
check_yaw = False
```

## Удержание позиции

В проекте есть несколько вариантов удержания:

### Удержание waypoint-а

Состояние `HOLD` используется для поисковых waypoint-ов, у которых задан
`hold_time`. В статическом fallback-маршруте `mission.py` используется:

```text
row_hold_sec = 1.5
```

В автоматическом маршруте `node.py` hold_time задается на последней точке и
на поворотах, если изменение направления больше `0.2` радиана. Величина
берется из параметра:

```text
search_hold_time_sec = 1.5
```

### Удержание текущей позиции

Функция:

```python
_hold_current_position_target()
```

замораживает target около последней известной позиции PX4:

```text
x = vehicle_local_position.x
y = vehicle_local_position.y
z = vehicle_local_position.z
yaw = vehicle_local_position.heading
```

Она используется, например, при подтверждении цели, при потере цели в
некоторых сценариях и при переходе в пассивный failsafe.

## Визуальное наведение и Offboard setpoint-ы

В режиме `FOLLOW_PERSON` управление все равно остается Offboard-управлением
через позицию и yaw. Детектор не управляет PX4 напрямую. Он публикует bbox в
`yolo/target_bbox`, а миссионный узел преобразует ошибку положения цели в
изменение `TargetState`.

В фазе выравнивания:

```python
_align_to_person_error()
```

target ставится в текущую позицию аппарата, а yaw постепенно изменяется:

```text
x = current x
y = current y
z = current z
yaw = heading + yaw_rate_cmd * control_period
```

Команда yaw-rate ограничивается `max_yaw_rate`.

В фазе движения вперед:

```python
_step_forward_for_bbox_growth()
```

target сдвигается вперед по текущему heading. Скорость задается параметром:

```text
follow_forward_speed = 0.35
```

При стандартном `control_period = 0.1 s` это дает шаг около:

```text
0.035 m за цикл
```

Если target слишком далеко впереди текущей позиции, используется ограничение:

```text
follow_target_lead_limit = 1.5
```

Тогда базовая точка берется из текущей позиции аппарата, а не из старого
target-а.

В коде также есть более общий метод `_update_person_follow_target()`, где
предусмотрены forward/lateral шаги и фильтрация yaw-rate. В текущем
сценарном обработчике `handle_follow_person()` основная последовательность
использует фазы:

```text
STOP_BEFORE_APPROACH
ALIGN_TO_TARGET
APPROACH_FORWARD
HOLD_AFTER_APPROACH
```

## Возврат домой

Возврат домой реализован состоянием:

```text
RETURN_HOME
```

Целевая точка возврата создается функцией:

```python
_prepare_return_home_waypoint()
```

Она задает:

```text
x = home_position.x
y = home_position.y
z = mission_plan.takeoff_waypoint.z
yaw = текущий target.yaw или home_position.yaw
```

То есть возврат происходит не на исходную высоту home position, а на рабочей
высоте миссии. После достижения home XY на рабочей высоте миссия переходит к
посадке, если не нужно возобновить поиск.

Возврат домой запускается в нескольких случаях:

- после завершения всех поисковых waypoint-ов, если `auto_land_on_finish=True`
  и home position известна;
- после завершения сценария взаимодействия с найденным человеком;
- при некоторых safety failure, если разрешен и возможен return-home recovery.

Для возврата после найденного человека может строиться A* маршрут эвакуации:

```python
_prepare_evacuation_route()
```

Он включается параметром:

```text
enable_evacuation_astar = True
```

Если A* маршрут не найден, код логирует предупреждение и использует прямой
return-home waypoint.

## Посадка

Посадка выполняется в состоянии:

```text
LAND
```

Функция:

```python
land()
```

публикует:

```text
command = VEHICLE_CMD_NAV_LAND
```

Команда отправляется один раз при входе в обработчик `handle_land()`:

```python
if not self.landing_command_sent:
    self.land()
    self.landing_command_sent = True
```

После этого узел ждет, пока PX4 через `VehicleStatus` сообщит, что аппарат
больше не armed:

```python
not self._is_armed()
```

Тогда состояние меняется:

```text
LAND -> FINISHED
```

## Safety checks

Централизованные проверки безопасности выполняются в начале каждого
`control_loop()`:

```python
safety_ok = self.check_safety()
```

Проверки не выполняются агрессивно для состояний:

```text
INIT
LAND
FAILSAFE
FINISHED
```

Для остальных состояний проверяется следующее.

### Наличие local position

Если local position еще не была получена и возраст узла больше:

```text
px4_data_timeout_sec = 1.0
```

вызывается failure:

```text
Local position data was not received
```

Если local position была получена, но стала старше `px4_data_timeout_sec`,
вызывается:

```text
Local position data timeout
```

### Наличие PX4 status

Если PX4 status был получен, но стал старше:

```text
px4_data_timeout_sec = 1.0
```

вызывается:

```text
PX4 status data timeout
```

### Failsafe от PX4

Если в `VehicleStatus` поле:

```text
failsafe = True
```

узел вызывает:

```text
PX4 reported failsafe state
```

### Таймаут waypoint-а

Для состояний:

```text
TAKEOFF
SEARCH
RETURN_HOME
```

если активный waypoint выполняется дольше:

```text
waypoint_timeout_sec = 120.0
```

вызывается failure:

```text
<STATE> waypoint timeout exceeded
```

### Таймаут сценария FOLLOW_PERSON

Для `FOLLOW_PERSON` проверяется:

```text
follow_person_action_timeout_sec = 90.0
```

При превышении вызывается:

```text
FOLLOW_PERSON action timeout exceeded
```

### Таймаут HOLD

Для `HOLD` лимит равен:

```text
current_search_waypoint.hold_time + hold_timeout_margin_sec
```

По умолчанию:

```text
hold_timeout_margin_sec = 1.0
```

### Таймаут vision

Если включен параметр:

```text
enable_vision = True
```

и в состояниях `SEARCH` или `HOLD` vision update старше:

```text
vision_timeout_sec = 1.0
```

вызывается:

```text
Vision data timeout
```

По умолчанию `enable_vision=False`, поэтому эта проверка выключена.

## Failsafe и восстановление через return-home

Все safety failure проходят через:

```python
handle_failure(reason)
```

Если можно безопасно вернуться домой, узел:

1. создает return-home waypoint;
2. переводит миссию в `RETURN_HOME`;
3. логирует причину как failure recovery.

Условие возможности return-home recovery находится в:

```python
_can_return_home_after_failure()
```

Для возврата должны одновременно выполняться условия:

```text
return_home_on_failure == True
home_position is not None
local position валидна
local position не старше px4_data_timeout_sec
vehicle status не старше px4_data_timeout_sec
аппарат armed
```

По умолчанию:

```text
return_home_on_failure = True
```

Если эти условия не выполнены, узел фиксирует текущую позицию через
`_hold_current_position_target()` и переходит в:

```text
FAILSAFE
```

В `FAILSAFE` он продолжает публиковать Offboard setpoint-ы, потому что
`FAILSAFE` входит в `_should_publish_offboard_setpoints()`. Агрессивного
восстановления в `handle_failsafe()` нет. Узел только ждет, пока PX4 status
покажет, что аппарат разоружен, после чего переходит в `FINISHED`.

## Параметры из `node.py`, влияющие на Offboard управление

Ниже перечислены параметры, которые напрямую влияют на Offboard-контур,
безопасность, возврат и посадку. Значения указаны по текущему `node.py`.

| Параметр | Значение по умолчанию | Назначение |
| --- | ---: | --- |
| `control_rate_hz` | `10.0` | Частота основного цикла управления. |
| `takeoff_height` | `7.0` | Рабочая высота как положительная величина; в waypoint превращается в отрицательный `z`. |
| `max_speed` | `3.0` | Максимальная скорость сдвига position setpoint-а. |
| `max_yaw_rate` | `0.5` | Максимальная скорость изменения yaw setpoint-а. |
| `waypoint_tolerance` | `0.4` | Радиус достижения waypoint-а по позиции. |
| `yaw_tolerance` | `0.3` | Допуск достижения yaw. |
| `px4_data_timeout_sec` | `1.0` | Таймаут свежести PX4 local position/status. |
| `waypoint_timeout_sec` | `120.0` | Максимальное время движения к waypoint-у. |
| `arming_timeout_sec` | `5.0` | Таймаут ожидания подтверждения arm/offboard перед fallback. |
| `hold_timeout_margin_sec` | `1.0` | Дополнительный запас к `hold_time` waypoint-а. |
| `return_home_on_failure` | `True` | Разрешает попытку возврата домой при safety failure. |
| `auto_land_on_finish` | `True` | После завершения маршрута возвращаться домой и садиться. |
| `enable_vision` | `False` | Включает проверку таймаута vision update для поиска. |
| `vision_timeout_sec` | `1.0` | Таймаут vision update, если `enable_vision=True`. |
| `enable_person_follow` | `True` | Разрешает переход из поиска в визуальное наведение. |
| `follow_person_action_timeout_sec` | `90.0` | Общий таймаут сценария `FOLLOW_PERSON`. |
| `follow_forward_speed` | `0.35` | Скорость forward-сдвига target-а при подходе к человеку. |
| `follow_slow_forward_speed` | `0.15` | Замедленная скорость при большом bbox. |
| `follow_target_lead_limit` | `1.5` | Ограничение отрыва target-а от текущей позиции при follow. |
| `follow_yaw_kp` | `0.8` | Пропорциональный коэффициент yaw-наведения по ошибке bbox. |
| `follow_yaw_kd` | `0.0` | Дифференциальный коэффициент yaw-наведения. |
| `follow_yaw_deadband` | `0.05` | Мертвая зона ошибки по горизонтали bbox. |
| `follow_yaw_unlock_deadband` | `0.12` | Порог выхода из locked yaw. |
| `follow_yaw_rate_filter_alpha` | `0.35` | Фильтрация yaw-rate команды в follow-логике. |
| `follow_lateral_gain` | `0.35` | Коэффициент бокового смещения target-а в follow-логике. |
| `follow_lateral_deadband` | `0.20` | Мертвая зона боковой коррекции. |
| `enable_evacuation_astar` | `True` | Разрешает A* маршрут возврата после взаимодействия с человеком. |
| `evacuation_grid_resolution` | `1.0` | Разрешение сетки A* возврата. |
| `evacuation_obstacle_margin` | `3.0` | Запас вокруг препятствий для A* возврата. |
| `evacuation_waypoint_spacing` | `3.0` | Интервал прореживания waypoint-ов A* маршрута. |
| `search_hold_time_sec` | `1.5` | Пауза на поворотах/последней точке автоматически построенного маршрута. |
| `search_route_publish_period_sec` | `2.0` | Период повторной публикации маршрута поиска для viewer-ов. |

## Ограничения текущей реализации

1. Управление реализовано только через position setpoint и yaw. Velocity,
   acceleration, attitude и body-rate режимы в `OffboardControlMode`
   отключены.

2. Скорость аппарата ограничивается не отдельным PX4 velocity setpoint-ом, а
   программным сглаживанием целевой позиции в ROS 2. Это делает поведение
   понятным в коде, но фактическая динамика все равно зависит от PX4.

3. В коде нет отдельной проверки успешной доставки каждой PX4-команды.
   Подтверждение arm/offboard оценивается по `VehicleStatus`.

4. Если `VehicleStatus` вообще не приходит, в `ARMING` есть fallback после
   `arming_timeout_sec`. Это полезно для совместимости, но в дипломе его
   лучше описывать как упрощение симуляционного стенда, а не как строгий
   промышленный safety-механизм.

5. Return-home recovery выполняется только если home position уже сохранена,
   локальная позиция валидна и свежая, статус PX4 свежий, аппарат armed и
   `return_home_on_failure=True`.

6. В `FAILSAFE` узел не строит новый маршрут и не отправляет отдельную команду
   посадки. Он удерживает текущий target и ждет разоружения по статусу PX4.

7. Возврат домой приводит аппарат к home `x/y` на рабочей высоте миссии, а не
   к исходной координате `home_position.z`. Посадка выполняется отдельной PX4
   командой `VEHICLE_CMD_NAV_LAND`.

8. README описывает общий сценарий проекта, но часть деталей в нем отличается
   от текущего кода. Например, в текущем `node.py` значение `takeoff_height`
   равно `7.0`, а не `5.0`; также публикация `mission/person_rescued` в
   `node.py` не обнаружена. Для описания Offboard управления в этом файле
   использованы значения и поведение из текущего кода.

## Краткая формулировка для диплома

В проекте управление квадрокоптером выполняется в режиме PX4 Offboard из
ROS 2 узла `offboard_takeoff`. Узел получает от PX4 локальную позицию и
статус аппарата, сохраняет домашнюю точку, публикует начальные setpoint-ы,
переводит PX4 в Offboard mode, отправляет команду arm и далее с частотой
10 Гц формирует позиционные `TrajectorySetpoint` сообщения. Управление
скоростью реализовано через сглаживание целевого положения: текущий setpoint
постепенно смещается к активному waypoint-у с ограничением линейного шага и
скорости изменения yaw. Высота задается в локальной системе PX4 NED, поэтому
рабочая высота `takeoff_height` преобразуется в отрицательную координату `z`.
В миссионном цикле предусмотрены проверки свежести данных PX4, флага failsafe,
таймаутов waypoint-ов и сценария сопровождения цели. При части отказов система
пытается вернуться домой, а если это невозможно, фиксирует текущую позицию и
переходит в пассивное состояние `FAILSAFE`.
