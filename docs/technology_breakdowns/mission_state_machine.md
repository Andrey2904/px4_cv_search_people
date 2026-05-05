# Миссионный конечный автомат

## Назначение

Миссионный конечный автомат управляет всем поведением БПЛА: ожиданием PX4,
взлетом, поиском людей, наведением на цель, возвратом, посадкой и обработкой
аварийных ситуаций.

Основная реализация находится в:

```text
src/offboard_takeoff/offboard_takeoff/node.py
```

Класс узла:

```text
OffboardTakeoff
```

Перечисление состояний:

```python
class MissionState(Enum):
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

## Общий принцип работы

Узел работает по таймеру:

```text
control_rate_hz = 10.0
```

То есть основной цикл управления вызывается примерно 10 раз в секунду.

Основная функция:

```python
control_loop()
```

В каждом цикле:

1. проверяется безопасность;
2. выбирается обработчик текущего состояния;
3. публикуются PX4 offboard setpoint-ы, если состояние требует управления.

## Основные PX4 topic-и

Узел публикует:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Узел подписывается на:

```text
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v2
yolo/target_bbox
```

## INIT

Начальное состояние.

Задачи:

- дождаться валидной локальной позиции PX4;
- сохранить home position;
- подготовить начальный target на точку взлета.

Переход:

```text
INIT -> WAIT_FOR_PX4
```

Условие перехода:

```text
получена валидная local position
```

## WAIT_FOR_PX4

PX4 требует, чтобы перед включением Offboard mode внешний узел некоторое время
публиковал setpoint-ы. Поэтому в этом состоянии дрон еще не армится, а узел
отправляет начальные setpoint-ы.

Количество начальных setpoint-ов:

```text
required_initial_setpoints = 30
```

При частоте 10 Гц это примерно 3 секунды.

Переход:

```text
WAIT_FOR_PX4 -> ARMING
```

Условие:

```text
отправлено 30 начальных setpoint-ов
```

После этого узел отправляет:

```text
OFFBOARD mode command
ARM command
```

## ARMING

Состояние ожидания подтверждения, что PX4:

- вооружил дрон;
- включил Offboard mode.

Параметр таймаута:

```text
arming_timeout_sec = 5.0
```

Переход:

```text
ARMING -> TAKEOFF
```

Условие:

```text
PX4 сообщает armed + offboard
```

Если статус PX4 недоступен, предусмотрен fallback после таймаута.

## TAKEOFF

Дрон летит к рабочей высоте.

Высота задается параметром:

```text
takeoff_height = 5.0
```

В PX4 local NED высота задается отрицательной координатой Z:

```text
z = -5.0
```

Движение к waypoint-у сглаживается:

```python
smooth_target_towards_waypoint()
```

Ограничения:

```text
max_speed = 0.8
max_yaw_rate = 0.4
waypoint_tolerance = 0.4
yaw_tolerance = 0.3
```

Переход:

```text
TAKEOFF -> SEARCH
```

Условие:

```text
точка взлета достигнута
```

## SEARCH

Основное состояние поиска.

Дрон летит по заранее заданному маршруту "змейкой". Маршрут описан в:

```text
src/offboard_takeoff/offboard_takeoff/mission.py
```

Функция:

```python
default_mission_plan()
```

Во время `SEARCH` постоянно проверяется, появился ли человек:

```python
_maybe_start_person_follow()
```

Возможные переходы:

```text
SEARCH -> HOLD
SEARCH -> FOLLOW_PERSON
SEARCH -> RETURN_HOME
```

`SEARCH -> HOLD` происходит, если waypoint имеет `hold_time`.

`SEARCH -> FOLLOW_PERSON` происходит после подтвержденного обнаружения
человека.

`SEARCH -> RETURN_HOME` происходит после завершения всех waypoint-ов маршрута.

## HOLD

Состояние удержания в точке маршрута.

Используется, когда waypoint требует паузу:

```text
hold_time = 1.5
```

Во время удержания дрон также продолжает проверять обнаружение человека.

Возможные переходы:

```text
HOLD -> SEARCH
HOLD -> FOLLOW_PERSON
```

## FOLLOW_PERSON

Состояние реакции на обнаруженного человека.

Внутри него есть собственные фазы:

```text
STOP_BEFORE_APPROACH
ALIGN_TO_TARGET
APPROACH_FORWARD
HOLD_AFTER_APPROACH
```

Кратко:

1. дрон зависает перед подлетом;
2. разворачивается по горизонтальной ошибке bbox;
3. летит вперед, пока bbox не увеличится;
4. зависает после подлета;
5. запускает эвакуационный маршрут.

Основные параметры:

```text
follow_detection_confirm_sec = 1.0
follow_stop_before_approach_sec = 1.0
follow_bbox_growth_target = 4.0
follow_forward_speed = 0.35
follow_hover_after_approach_sec = 5.0
follow_person_action_timeout_sec = 90.0
```

Переход:

```text
FOLLOW_PERSON -> RETURN_HOME
```

После завершения реакции на человека строится A* маршрут эвакуации.

## RETURN_HOME

Состояние возврата к home position или зоне эвакуации.

Раньше дрон летел домой напрямую. После добавления A* логика стала такой:

1. если A* маршрут найден, `RETURN_HOME` проходит по списку waypoint-ов;
2. если A* маршрут не найден, используется прямой waypoint home;
3. после достижения последней точки выполняется дальнейшее действие.

Если это возврат после спасения человека:

```text
RETURN_HOME -> SEARCH
```

Если миссия полностью завершена:

```text
RETURN_HOME -> LAND
```

## LAND

Состояние посадки.

Узел отправляет PX4 команду:

```text
VEHICLE_CMD_NAV_LAND
```

Переход:

```text
LAND -> FINISHED
```

Условие:

```text
PX4 сообщает, что дрон разоружен
```

## FAILSAFE

Состояние аварийного удержания.

В него можно попасть при ошибках безопасности, если невозможно безопасно
вернуться домой.

Например:

- нет local position;
- PX4 сообщает failsafe;
- waypoint timeout;
- слишком долгое состояние FOLLOW_PERSON.

Если возможно, при ошибке узел сначала пытается вернуться домой:

```python
handle_failure()
```

Если возврат невозможен, дрон удерживает текущую позицию и переходит в
`FAILSAFE`.

## FINISHED

Финальное состояние.

В нем основной цикл больше не выполняет миссионных действий.

## Проверки безопасности

Центральная функция:

```python
check_safety()
```

Основные параметры:

```text
px4_data_timeout_sec = 1.0
waypoint_timeout_sec = 45.0
hold_timeout_margin_sec = 1.0
return_home_on_failure = true
```

Проверяется:

- приходят ли данные local position;
- приходят ли данные vehicle status;
- не сообщил ли PX4 failsafe;
- не превышен ли таймаут waypoint-а;
- не завис ли режим `FOLLOW_PERSON`;
- не зависло ли удержание в `HOLD`.

## Упрощенная схема переходов

```text
INIT
  -> WAIT_FOR_PX4
  -> ARMING
  -> TAKEOFF
  -> SEARCH
       -> HOLD -> SEARCH
       -> FOLLOW_PERSON -> RETURN_HOME -> SEARCH
       -> RETURN_HOME -> LAND -> FINISHED

Любое активное состояние
  -> RETURN_HOME или FAILSAFE при ошибке
```

## Краткое описание для диплома

Миссия БПЛА реализована в виде конечного автомата, где каждое состояние
отвечает за отдельный этап полета: инициализацию, подготовку PX4 Offboard,
взлет, поиск, удержание в точках маршрута, реакцию на обнаруженного
пострадавшего, возврат в зону эвакуации, посадку и обработку аварийных
ситуаций. Такой подход упрощает контроль логики, позволяет явно задавать
условия переходов и повышает надежность системы за счет централизованных
проверок безопасности и таймаутов.

