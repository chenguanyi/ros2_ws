# ROS 2 飞行任务工作空间

本仓库是一个面向飞行任务开发的 ROS 2 工作空间，当前按“基础能力对象 + 任务节点 + 用户 launch”的方式组织。底层包只保留可复用能力，具体比赛题目和任务流程写成独立任务节点。



固定航点任务是当前主要入口：

```bash
ros2 launch my_launch fixed_waypoint.launch.py
```

G 题植保任务入口：

/dev/camera_decxin 是向下摄像头，用于识别绿色/白布格

/dev/camera_wdr5m 是侧向摄像头，用于识别杆塔条形码

```bash
ros2 launch my_launch plant_protection.launch.py
```

D 题立体货架盘点任务入口：

```bash
ros2 launch my_launch warehouse_inventory.launch.py mission_mode:=inventory
ros2 launch my_launch warehouse_inventory.launch.py mission_mode:=target
```

`inventory` 会遍历 24 个货架二维码，完整成功后更新：

```text
mylog/warehouse_inventory/latest_success.json
```

运行过程中会持续更新：

```text
mylog/warehouse_inventory/latest_progress.json
```

最后一个货架点处理完成并进入返航前会立即更新 `latest_success.json`；如果有漏扫，文件也会生成，`complete=false` 且 `missing_coords` 会列出缺失位置。

`target` 会读取最新成功盘点记录，等待右侧摄像头扫到目标二维码编号，再按记录中的标签位生成完整固定安全路线，并一次性发布到 `/d_task/planned_route`；到目标标签后继续视觉确认二维码，确认后打光并去终点降落。

`/d_task/planned_route` 是一段较长 JSON，`ros2 topic echo` 可能显示 `...`；查看完整内容使用：

```bash
ros2 topic echo --full-length /d_task/planned_route
```

任务 launch 会自动把 ROS 日志保存到工作区的任务目录：

```text
mylog/fixed_waypoint/
mylog/plant_protection/
mylog/warehouse_inventory/
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

## 主数据流

- `my_carto_pkg` / `bluesea2` 提供定位、建图、TF。
- `uart_to_stm32` 负责串口收发，发布 `/height`，并转发 `/target_velocity`。
- `pid_control_pkg` 订阅 `/target_position`、`/height` 和 TF，发布 `/target_velocity`。
- `activity_control_pkg` 的任务节点发布 `/target_position`、`/active_controller`、`/mission_complete` 和辅助控制命令。
- `visual_pkg` 只发布 `/camera/image_raw`，具体识别逻辑由任务节点或任务专属辅助类实现。

## 基础话题

- `/target_position`: 航点目标，格式 `[x_cm, y_cm, z_cm, yaw_deg]`。
- `/active_controller`: `2` 表示允许 PID/UART 执行任务速度，`3` 表示停止任务速度。
- `/target_velocity`: PID 输出给 UART 的速度。
- `/mission_complete`: 任务结束事件。
- `/arm_command`: 机械臂语义命令。
- `/magnet_command`: 电磁铁语义命令。
- `/signal_command`: 通用开关命令。
- `/camera/image_raw`: 原始相机画面。

PID 和 UART 只关心这些基础话题，不承载具体比赛题目的流程。

## 包职责

### `activity_control_pkg`

基础能力对象和任务节点所在包。核心对象放在：

```text
src/activity_control_pkg/include/activity_control_pkg/core/
src/activity_control_pkg/src/core/
```

- `WaypointNavigator`: 飞航点、到点判断、hold、控制器启停。
- `AuxController`: 发布 `/arm_command`、`/magnet_command`、`/signal_command`。
- `SensorCache`: 缓存 `/height`、`/height_raw`、`/laser_array/ground_height`、`/laser_array/obstacle_height`。
- `ImageCache`: 缓存 `/camera/image_raw` 最新图像。

任务节点放在 `src/activity_control_pkg/src/tasks/`，当前示例：

- `fixed_waypoint_task_node`
- `plant_protection_task_node`

### `my_launch`

面向最终使用者的启动入口。当前保留：

- `fixed_waypoint.launch.py`
- `plant_protection.launch.py`
- `warehouse_inventory.launch.py`

以后不同题目可以新增不同 launch，例如：

- `pillar_task.launch.py`

### `visual_pkg`

只做相机源：

- `camera_source_node`
- `/camera/image_raw`

旧视觉检测逻辑已移除。任务需要识别条码、颜色、圆环等内容时，应在任务内部或任务专属辅助类里处理图像。

### `uart_to_stm32`

只做串口协议桥：

- 接收 STM32 数据。
- 发布高度/状态。
- 发送速度帧。
- 发送机械臂、电磁铁和 signal 控制帧。
- 发送任务完成帧。

### `pid_control_pkg`

位置 PID 控制器：

- 输入 `/target_position`、`/height`、TF。
- 输出 `/target_velocity`。

## 新任务开发

新任务不要改 PID、UART 或相机源。推荐结构：

```text
src/activity_control_pkg/src/tasks/plant_protection_task_node.cpp
src/my_launch/launch/plant_protection.launch.py
```

新增任务时：

1. 在 `src/activity_control_pkg/src/tasks/` 下写一个任务节点。
2. 在任务节点里按需实例化 `WaypointNavigator`、`AuxController`、`SensorCache`、`ImageCache`。
3. 在 `src/activity_control_pkg/CMakeLists.txt` 中注册 executable，并链接 `mission_core`。
4. 在 `src/my_launch/launch/` 下新增用户 launch，把本任务常调参数放在 launch 顶部。

不要把比赛题目的规则写进 PID、UART、相机源或 `WaypointNavigator`。

## G 题植保任务

当前已提供：

```text
plant_protection_task_node.cpp
plant_protection.launch.py
```

任务内部负责：

- 建立 400 cm x 500 cm 作业区模型。
- 按 50 cm 网格生成待撒药格子。
- 起飞后先飞到条码识别点，识别成功后用 LED 持续重复显示条码数字。
- 从 A/21 格开始按最短覆盖路径撒药。
- 遇到发挥部分的 3-4 个白/灰遮挡格时，向下视觉自动跳过明显非绿色格。
- 飞到每个撒药点时调用 `flight_.goTo(...)`。
- 到点后用 `aux_.setMagnet(true)` 控制向下激光闪烁，再关闭。
- `signal` 位复用为 LED，条码识别结果来自 `/plant_protection/barcode_value`。
- 条码数字转换为半径后，任务自己计算圆周降落点，再调用 `flight_.goTo(...)`。

`WaypointNavigator` 不知道 G 题是什么，它只负责飞到指定点。

## D 题立体货架盘点任务

当前已提供：

```text
warehouse_inventory_task_node.cpp
warehouse_inventory.launch.py
```

任务内部负责：

- 固定建立 400 cm x 500 cm 场地、2 个货架、A/B/C/D 面和 24 个二维码扫描点。
- 右侧摄像头固定用于二维码识别和精调，二维码中心 `image_dx` 控制机体 `x`，`image_dy` 控制机体 `z`，机体 `y` 靠离板 60 cm 的航点保持。
- 面间切换使用安全路线，扫描点和绕行点离板至少 60 cm。
- `inventory` 模式穷举面顺序与蛇形扫描变体，选最短安全路线后遍历 24 个点。
- 每识别一个二维码后，水平激光点亮约 0.5 秒，LED 亮约 1 秒，并发布盘点结果。
- 最后一个货架点完成或超时后立即保存 `inventory_<timestamp>.json` 并更新 `latest_success.json`；漏扫时也会落盘，靠 `complete=false` 和 `missing_coords` 排查。
- `target` 模式读取 `latest_success.json`，用右侧摄像头扫到的二维码编号查表得到标签位，使用固定对称绕行点生成安全路线，并同步发布到 `/d_task/planned_route` 与 `/warehouse_inventory/route`；到标签后视觉确认二维码，打光后去终点降落。

## 工程边界

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

更详细的设计和开发说明见 `docs/mission_framework_plan.md`。
