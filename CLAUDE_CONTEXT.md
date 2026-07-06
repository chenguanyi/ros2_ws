# Claude 工作区上下文

## 项目目标

- 当前重点：继续消缺电赛第一题 `diansai_first`，保持任务/PID 统一使用 `/height`，由 `uart_to_stm32` 在节点内部选择高度来源并发布 `/height`；继续排查机械臂实际不伸出的问题。

## 当前状态

- 已确认架构约定：所有任务节点和 PID 控制只消费 `/height`；`height_source` 只用于 `uart_to_stm32` 在 `serial_raw` 与 `laser_ground` 间选择输入，再发布统一 `/height`。
- 本地 `my_launch/launch/diansai_first.launch.py` 已设置 `HEIGHT_SOURCE = "laser_ground"`，任务参数仍是 `height_topic: "/height"`，并启动 `laser_array_ground.launch.py`。
- 已将当前 worktree 的 `my_launch/launch/diansai_first.launch.py` 单文件同步到远端 `/home/orangepi/ros2_ws/src/my_launch/launch/diansai_first.launch.py`，并在远端执行 `colcon build --symlink-install --packages-select my_launch` 成功。
- 已验证远端 source 与 install 空间的 `diansai_first.launch.py` 均为 `HEIGHT_SOURCE = "laser_ground"`。
- 已短启动正式 launch 验证：最新远端 `uart_to_stm32` 日志打印 `Height source=laser_ground laser_topic=/laser_array/ground_height`；`laser_array_ground_node` 有高度输出；任务节点仍等待/使用 `/height`。
- 机械臂链路日志边界：任务节点已发布 `arm=1 magnet=1 signal=0`，`uart_to_stm32` 已成功发送 0x33 `[arm=1, magnet=1, signal=0]`。若机械臂仍不动，问题优先怀疑 STM32 协议解析/执行、供电、接线、执行器或 `arm=1` 语义，而不是 ROS 任务或高度架构。
- 短启动时蓝海雷达出现 `send commands to lidar but no answer`，这是独立 lidar 通信问题，不影响本次高度源验证结论。

## 重要约定

- `diansai_first` 任务逻辑放在独立任务目录中，复用底层 `WaypointNavigator`、`AuxController`、PID 视觉接管、TF、`/height`、`visual_pkg`。
- 不把任务流程写入 `pid_control_pkg`、`uart_to_stm32` 或 `visual_pkg`，除非是为共享基础设施增加日志/诊断。
- 高度架构约定：任务/PID 一律使用 `/height`；不要在任务层或 PID 层直接订阅 `/laser_array/ground_height` 来切换高度源。切源只改 `uart_to_stm32` 的 `height_source` 参数/launch 传参。
- `arm=1` 放下/伸出，`arm=0` 收起；`magnet=1` 吸附，`magnet=0` 释放。
- 校准 launch：`ros2 launch my_launch diansai_first_alignment_check.launch.py`（不启动 UART，只测对准和速度方向）。
- 正式 launch：`ros2 launch my_launch diansai_first.launch.py`。
- 本地到远端单向镜像；`REMOTE_SRC_DIR` 保持 `/home/orangepi/ros2_ws/src`。
- 修改哪个 ROS 包就重新编译哪个包。
- 引用行号前必须核实 `wc -l`；行号超出时立即报告无效。
- `/push-remote` 同步的是主 checkout（`/home/ubuntu/remote-board-dev/`），不是当前 worktree；改完 worktree 后必须确认推送的是 worktree 文件，或者直接把目标文件 rsync 到远端。

## 下一步

1. 单独单测机械臂 UART/STM32/硬件链路：启动 `uart_to_stm32` 后直接发布 `/arm_command` 为 1/0，观察 UART 0x33 日志和机械臂实际动作。
2. 若 UART 日志仍显示成功发送 0x33 但机械臂不动，转查 STM32 固件解析、协议字段顺序、`arm=1` 极性/语义、接线、供电和执行器。
3. 如继续跑正式任务，先确认蓝海雷达通信正常，避免 `send commands to lidar but no answer` 影响定位。

## 最近更新

- 2026-07-06：确认并固化 `/height` 统一高度架构；已同步并重建远端 `diansai_first.launch.py`，验证 `uart_to_stm32` 运行时使用 `laser_ground` 发布 `/height`，机械臂问题转入 STM32/硬件链路单测。
