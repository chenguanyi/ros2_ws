# ROS 2 工作空间源码结构

当前源码按“基础能力对象 + 任务节点 + 用户 launch”组织。

## 常用启动

固定航点任务：

```bash
ros2 launch my_launch fixed_waypoint.launch.py
```

G 题植保任务：

```bash
ros2 launch my_launch plant_protection.launch.py
```

任务日志会保存到工作区 `mylog/<任务名>/` 目录。

相机源单独启动：

```bash
ros2 launch visual_pkg camera_source.launch.py
```

## 主数据流

- `my_carto_pkg` / `bluesea2` 提供定位、建图、TF。
- `uart_to_stm32` 负责串口收发，并发布 `/height`、转发 `/target_velocity`。
- `pid_control_pkg` 订阅 `/target_position`、`/height` 和 TF，发布 `/target_velocity`。
- `activity_control_pkg` 的任务节点发布 `/target_position`、`/active_controller`、`/mission_complete` 和 aux 命令。
- `visual_pkg` 只发布 `/camera/image_raw`，具体识别由任务自己实现。

## 包职责

### `activity_control_pkg`

核心对象在 `include/activity_control_pkg/core/` 和 `src/core/`：

- `WaypointNavigator`：飞航点、到点判断、hold、控制器启停。
- `AuxController`：发布 `/arm_command`、`/magnet_command`、`/signal_command`。
- `SensorCache`：缓存高度和激光基础数据。
- `ImageCache`：缓存 `/camera/image_raw` 最新图像。

任务节点放在 `src/tasks/`。当前示例：

- `fixed_waypoint_task_node`
- `plant_protection_task_node`

### `my_launch`

面向最终使用者的启动入口。当前保留：

- `fixed_waypoint.launch.py`
- `plant_protection.launch.py`

以后不同题目新增不同 launch，例如：

- `pillar_task.launch.py`

### `visual_pkg`

只做相机源：

- `camera_source_node`
- `/camera/image_raw`

旧视觉检测逻辑已移除。任务需要识别时，在任务内部或任务专属辅助类里处理图像。

### `uart_to_stm32`

只做串口协议桥：

- 接收 STM32 数据。
- 发布高度/状态。
- 发送速度帧。
- 发送 aux 控制帧。
- 发送任务完成帧。

不承载具体比赛任务流程。

### `pid_control_pkg`

位置 PID 控制器：

- 输入 `/target_position`、`/height`、TF。
- 输出 `/target_velocity`。

## 新任务开发原则

新任务不要改 PID、UART 或相机源。

新建任务时：

1. 在 `activity_control_pkg/src/tasks/` 新增一个任务节点。
2. 在任务节点里按需实例化 `WaypointNavigator`、`AuxController`、`SensorCache`、`ImageCache`。
3. 在 `activity_control_pkg/CMakeLists.txt` 注册新 executable。
4. 在 `my_launch/launch/` 新增一个用户 launch，并把该题常调参数放在 launch 顶部。

详细说明见：

```text
docs/mission_framework_plan.md
```
