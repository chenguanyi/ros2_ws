# 野生动物巡查飞机端接口

启动飞机端：

```bash
ros2 launch my_launch wildlife_patrol.launch.py
```

## 状态 ACK

飞机端在 `/wildlife/status` 发布 `std_msgs/msg/String`。该话题使用 reliable、transient-local QoS，地面站可在稍后启动并立即读取最后状态；飞机端也每秒发布一次心跳。

消息为 JSON，例如：

```json
{
  "phase": "WAIT_FLIGHT_STATE",
  "ready": false,
  "route_valid": true,
  "route_accepted": true,
  "waypoint_count": 63,
  "current_waypoint_index": 0,
  "note": "route_accepted",
  "error": ""
}
```

`phase` 依次为 `WAIT_ROUTE`、`WAIT_FLIGHT_STATE`、`FLYING` 和 `HOLDING`。
`current_waypoint_index` 从 1 开始，未起飞时为 0。路线格式错误时，`note` 为 `route_rejected`，`error` 给出原因。

航点高度可以随航线变化，允许使用 `z=0` 的降落航点；只有首航点必须是固定起飞点 `(0,0,120,0)`。航向仍默认要求为 `0°`。

地面站在收到 `route_accepted=true` 和 `note=route_accepted` 后，才应显示“飞机已确认路线”。

任务结束时发布一次 `/mission_complete`（`std_msgs/msg/Empty`）。若路线最后航点高度不超过
`10 cm`，到达该点后立即完成；若最后航点高于 `10 cm`，飞机会在相同 XY 位置下降到
`10 cm`，到达后再发布完成消息。

动物事件只接受置信度不低于 `0.70` 的检测，且同一类别必须连续检测到 3 帧才发布一次 `/animal`。
目标消失后该类别确认状态清零；目标持续存在不会重复刷 `/animal`。可在 launch 中调整：

```bash
ros2 launch my_launch wildlife_patrol.launch.py \
  animal_confidence_threshold:=0.70 animal_confirm_frames:=3
```

联调：

```bash
ros2 topic echo /wildlife/status
ros2 topic echo /animal
```

## 日志

面阵激光高度摘要默认每 5 秒输出一次。调试时可以覆盖：

```bash
ros2 launch my_launch wildlife_patrol.launch.py laser_log_period_sec:=1.0
```
