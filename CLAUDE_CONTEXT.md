# Claude 工作区上下文

## 项目目标

- 建立一套通用的“本地 WSL 编辑 + rsync 单向同步 + SSH 远端 Linux 开发板编译/运行”的开发模板。
- 当前项目已切换为：以远端 Orange Pi 的 ROS2 工作区 `/home/orangepi/ros2_ws` 为初始可信源，先拉到本地，再由本地作为源码可信源同步回远端构建/运行。

## 当前状态

- 工作目录：`/home/ubuntu/remote-board-dev`。
- 已成功 SSH 免密连接：`orangepi@10.249.41.19`，远端主机名 `orangepi5max`。
- 远端 ROS 环境：`/opt/ros/humble`，`colcon` 位于 `/usr/bin/colcon`。
- 已从远端 `/home/orangepi/ros2_ws/src` 拉取 ROS2 源码到本地项目根目录。
- 本地已发现 ROS2 包：
  - `activity_control_pkg`
  - `circle_detector`
  - `laser_array_pkg`
  - `my_carto_pkg`
  - `my_launch`
  - `pid_control_pkg`
  - `pillar_detector_pkg`
  - `serial_comm`
  - `uart_to_stm32`
  - `visual_pkg`
- 已新增安全脚本：`scripts/rsync-from-remote.sh`，用于远端到本地 bootstrap，默认不使用 `--delete`。
- 已更新 `scripts/rsync-to-remote.sh`：
  - 非 dry-run 推送要求 `LOCAL_SOURCE_IS_CANONICAL=1`。
  - 排除 `.git/`、`.gitignore`、`.claude/`、`CLAUDE_CONTEXT.md`、swap 文件、`docs/`、`scripts/`、`build/`、`install/`、`log/` 等。
- 已创建本地私密配置 `scripts/remote.env`：
  - `REMOTE_SRC_DIR=/home/orangepi/ros2_ws/src`
  - `REMOTE_WORK_DIR=/home/orangepi/ros2_ws`
  - `LOCAL_SOURCE_IS_CANONICAL=1`
  - `REMOTE_BUILD_CMD="source /opt/ros/humble/setup.bash && colcon build --symlink-install"`
  - `REMOTE_RUN_CMD="source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list"`
- 已执行本地到远端 dry-run 和真实同步；没有文件变更、没有删除。
- 已执行 `scripts/remote-build.sh`，远端 `colcon build --symlink-install` 成功：12 个 packages finished。
- 已执行 `scripts/remote-run.sh`，smoke command `ros2 pkg list` 成功。
- 用户确认允许远端 `README.md` 覆盖模板根目录 `README.md`；模板说明主要保留在 `docs/remote-dev.md` 和脚本中。

## 重要约定

- 当前从现在起：本地 `/home/ubuntu/remote-board-dev` 是源码可信源。
- 远端 `/home/orangepi/ros2_ws/src` 是本地源码的镜像，正常开发中不要在远端手工改源码。
- 同步策略为本地到远端的单向镜像，`rsync-to-remote.sh` 使用 `--delete`，但已经用 `LOCAL_SOURCE_IS_CANONICAL` 做保护。
- `scripts/remote.env` 是本地私密配置，已由 `.gitignore` 排除，不应提交或同步。
- ROS2 构建应在远端工作区根目录 `/home/orangepi/ros2_ws` 执行，不是在 `src` 目录执行。
- `REMOTE_SRC_DIR` 保持窄路径 `/home/orangepi/ros2_ws/src`，不要设为 `/home/orangepi` 或 `/home/orangepi/ros2_ws`。
- `docs/` 和 `scripts/` 被排除在推送之外，避免把本地开发模板工具同步进远端 ROS2 `src`。

## 下一步

1. 如果要修改功能代码，直接编辑本地 ROS2 包，然后运行：
   - `scripts/rsync-to-remote.sh --dry-run`
   - `scripts/remote-build.sh`
   - `scripts/remote-run.sh` 或将 `REMOTE_RUN_CMD` 改为实际节点/launch 命令后再运行。
2. 可根据实际应用把 `scripts/remote.env` 中的 `REMOTE_RUN_CMD` 从 `ros2 pkg list` 替换为真实启动命令，例如 `ros2 launch ...` 或 `ros2 run ...`。
3. 如果需要同步整个远端 `ros2_ws` 根目录中的 `README.md`、`docs/`、`scripts/`、数据文件等，需要重新设计本地目录布局；当前只管理远端 `ros2_ws/src`。

## 最近更新

- 2026-07-05：完成 SSH 免密连接、远端 ROS2 工作区探测、从远端 `ros2_ws/src` 拉取到本地、安全推送 dry-run/真实同步、远端 `colcon build` 和 `ros2 pkg list` smoke run 验证。
