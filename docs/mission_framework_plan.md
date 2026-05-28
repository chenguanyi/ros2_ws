# 飞行任务框架使用与开发手册

这套工程现在按“基础能力对象 + 任务节点”的方式组织。

简单说：底层对象只负责会重复使用的能力，比赛题目自己写成一个任务节点。任务节点需要飞到哪里，就实例化 `WaypointNavigator` 并调用 `goTo()`；需要机械臂、电磁铁或 signal，就实例化 `AuxController`；需要画面，就订阅相机源或使用 `ImageCache`。

## 1. 队员日常怎么启动

固定航点任务入口：

```bash
ros2 launch my_launch fixed_waypoint.launch.py
```

G 题植保任务入口：

```bash
ros2 launch my_launch plant_protection.launch.py
```

任务 launch 会自动把 ROS 日志保存到工作区的任务目录：

```text
mylog/fixed_waypoint/
mylog/plant_protection/
```

常用参数：

```bash
ros2 launch my_launch fixed_waypoint.launch.py use_rviz:=true
ros2 launch my_launch fixed_waypoint.launch.py use_camera:=false
ros2 launch my_launch fixed_waypoint.launch.py height_source:=serial_raw
```

固定航点列表在 `src/my_launch/launch/fixed_waypoint.launch.py` 顶部的 `MISSION_STEPS` 中修改：

```python
MISSION_STEPS = [
    {"pose": (0.0, 0.0, 80.0, 0.0), "arm": 0, "magnet": 0, "signal": 0, "hold_sec": 0.0},
    {"pose": (100.0, 0.0, 80.0, 0.0), "arm": 0, "magnet": 0, "signal": 1, "hold_sec": 1.0},
]
```

字段含义：

- `pose`: `(x_cm, y_cm, z_cm, yaw_deg)`，单位分别是 cm、cm、cm、度。
- `arm`: 机械臂动作，0 或 1。
- `magnet`: 电磁铁动作，0 或 1。
- `signal`: 通用 signal 输出，0 或 1。
- `hold_sec`: 到达该航点后等待多久再继续。

## 2. 当前保留的基础话题

下游协议没有变：

- `/target_position`: 航点目标，格式 `[x_cm, y_cm, z_cm, yaw_deg]`。
- `/active_controller`: `2` 表示允许 PID/UART 执行任务速度，`3` 表示停止任务速度。
- `/target_velocity`: PID 输出给 UART 的速度。
- `/mission_complete`: 任务结束事件。
- `/arm_command`: 机械臂语义命令。
- `/magnet_command`: 电磁铁语义命令。
- `/signal_command`: 通用开关命令。
- `/camera/image_raw`: 原始相机画面。

PID 和 UART 只关心这些基础话题，不写具体比赛题目的流程。

## 3. 基础对象怎么分工

核心对象放在：

```text
src/activity_control_pkg/include/activity_control_pkg/core/
src/activity_control_pkg/src/core/
```

### `WaypointNavigator`

负责最基础的“飞航点”：

- 发布 `/target_position`。
- 发布 `/active_controller`。
- 订阅 `/height`。
- 查询 TF：默认 `map -> laser_link`。
- 判断是否到达目标点。
- 处理到点后的 hold。

任务里这样用：

```cpp
WaypointNavigator flight(*this);
flight.goTo(100.0, 0.0, 120.0, 0.0);

if (flight.isReached()) {
  flight.hold(1.0);
}
```

### `AuxController`

负责机械臂、电磁铁、signal 三类语义动作：

```cpp
AuxController aux(*this);
aux.setArm(true);
aux.setMagnet(false);
aux.setSignal(true);
aux.setAll(0, 0, 1);
```

### `SensorCache`

缓存基础传感器值，供需要感知的任务读取：

- `/height`
- `/height_raw`
- `/laser_array/ground_height`
- `/laser_array/obstacle_height`

### `ImageCache`

缓存 `/camera/image_raw` 最新一帧。新任务需要识别条码、颜色、圆环等内容时，应在自己的任务逻辑里处理图像，不要把识别逻辑塞回相机节点。

## 4. 怎么新增一个任务

推荐结构：

```text
src/activity_control_pkg/src/tasks/plant_protection_task_node.cpp
src/my_launch/launch/plant_protection.launch.py
```

任务节点基本写法：

```cpp
class PlantProtectionTaskNode : public rclcpp::Node
{
public:
  PlantProtectionTaskNode()
  : Node("plant_protection_task_node"),
    flight_(*this),
    aux_(*this),
    sensors_(*this),
    image_(*this)
  {
    // 声明本任务自己的参数
    // 创建 timer，在 timer 里推进任务状态机
  }

private:
  WaypointNavigator flight_;
  AuxController aux_;
  SensorCache sensors_;
  ImageCache image_;
};
```

新增任务时只需要做三件事：

1. 在 `src/tasks/` 下写一个任务节点。
2. 在 `activity_control_pkg/CMakeLists.txt` 里添加一个 executable，并链接 `mission_core`。
3. 在 `my_launch/launch/` 下写一个该任务专用 launch，把本任务常调参数放在 launch 顶部。

不要把比赛题目的规则写进 PID、UART、相机源或 `WaypointNavigator`。

## 5. G 题植保任务

当前 G 题已新增：

```text
plant_protection_task_node.cpp
plant_protection.launch.py
```

任务内部负责：

- 建立 400 cm x 500 cm 作业区模型。
- 按 50 cm 网格生成待撒药格子。
- 从 A 格开始生成覆盖路径。
- 遇到发挥部分的 3-4 个遮挡格时，从任务参数中排除这些格。
- 飞到每个撒药点时调用 `flight_.goTo(...)`。
- 到点后用 `aux_.setMagnet(true)` 控制向下激光闪烁，再关闭。
- `signal` 位复用为 LED，`barcode_value` 参数用于条码数字预留显示。
- 条码识别后续可放在任务内部或任务专属视觉辅助类中。
- 条码数字转换为半径后，任务自己计算圆周降落点，再调用 `flight_.goTo(...)`。

`WaypointNavigator` 不知道 G 题是什么，它只负责飞到指定点。

## 6. 当前工程边界

保留：

- `activity_control_pkg`: 基础能力对象 + 任务节点。
- `pid_control_pkg`: 位置 PID。
- `uart_to_stm32`: 串口协议桥。
- `visual_pkg`: 原始相机源。
- `my_launch`: 面向最终使用者的任务 launch。

已清理：

- pluginlib/XML/runner 动态插件方案。
- 旧 waypoint/fixed waypoint 重复节点。
- 旧 `route_target_publisher` 找柱抓放历史链路。
- 旧 `visual_node` 圆环检测链路。

以后如果要恢复找柱/抓放，应按新结构写成 `pillar_task_node.cpp` 和 `pillar_task.launch.py`，不要再把流程塞进底层包。
