# WSL 本地编辑 + 远端 Linux 开发板构建流程

这个目录是一个通用模板，不绑定 KR260 或任何具体开发板。它的目标是：本地 WSL 负责编辑源码，远端 Linux 开发板负责真实编译、运行和验证。

当前模板支持两种阶段：

1. **bootstrap 阶段**：远端已有 ROS2 工作区时，先从远端 `REMOTE_SRC_DIR` 拉取到本地。
2. **正常开发阶段**：确认本地源码完整后，本地成为可信源，再同步到远端构建/运行。

## 一次性配置

1. 复制配置模板：

   ```sh
   cp scripts/remote.env.example scripts/remote.env
   ```

2. 编辑 `scripts/remote.env`，填入开发板 SSH 信息、远端源码目录、构建命令和运行命令。

   `scripts/remote.env` 是本机私密配置，已被 `.gitignore` 排除。

ROS2 工作区常见配置：

```sh
REMOTE_SRC_DIR="/home/orangepi/ros2_ws/src"
REMOTE_WORK_DIR="/home/orangepi/ros2_ws"
LOCAL_SOURCE_IS_CANONICAL="0"
REMOTE_BUILD_CMD="source /opt/ros/humble/setup.bash && colcon build --symlink-install"
REMOTE_RUN_CMD="source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list"
```

`LOCAL_SOURCE_IS_CANONICAL="0"` 表示本地还不是可信源，真实的本地到远端推送会被拒绝。完成远端到本地 bootstrap 并确认本地源码完整后，再改为 `1`。

## 远端已有 ROS2 工作区时的 bootstrap 流程

首次使用时，先检查远端目录和 ROS 环境：

```sh
ssh -p 22 orangepi@10.249.41.19 'hostname && whoami'
ssh -p 22 orangepi@10.249.41.19 'ls -la /home/orangepi/ros2_ws && ls -la /home/orangepi/ros2_ws/src'
ssh -p 22 orangepi@10.249.41.19 'ls -d /opt/ros/* 2>/dev/null; command -v colcon || true'
```

然后先 dry run：

```sh
scripts/rsync-from-remote.sh --dry-run
```

确认源是远端 `REMOTE_SRC_DIR`、目标是本地项目目录，并且没有删除行为后，再真实拉取：

```sh
scripts/rsync-from-remote.sh
```

拉取完成后检查本地 ROS2 包：

```sh
find . -maxdepth 3 -name package.xml -print
```

确认本地源码完整后，将 `scripts/remote.env` 中：

```sh
LOCAL_SOURCE_IS_CANONICAL="1"
```

之后进入正常开发阶段。

## 常用命令

### 预览本地到远端同步

首次切换到本地可信源后，必须先 dry run：

```sh
scripts/rsync-to-remote.sh --dry-run
```

也可以用环境变量：

```sh
DRY_RUN=1 scripts/rsync-to-remote.sh
```

### 同步本地源码到远端

```sh
scripts/rsync-to-remote.sh
```

同步是单向镜像：本地覆盖远端，并使用 `rsync --delete` 删除远端源码目录里本地不存在的文件。只有 `LOCAL_SOURCE_IS_CANONICAL=1` 时才允许真实推送。不要在远端源码目录里手工改源码。

### 同步并远端编译

```sh
scripts/remote-build.sh
```

脚本会先同步，再通过 SSH 进入 `REMOTE_WORK_DIR` 执行 `REMOTE_BUILD_CMD`。远端 stdout/stderr 会直接显示在本地终端，用于本地修改和迭代。

对于 ROS2，`REMOTE_WORK_DIR` 通常是工作区根目录，例如 `/home/orangepi/ros2_ws`，而不是 `src` 目录。

### 远端运行

不重新同步，直接运行：

```sh
scripts/remote-run.sh
```

运行前先同步：

```sh
scripts/remote-run.sh --sync
```

### 打开远端 shell

```sh
scripts/remote-shell.sh
```

## 目录约定

- bootstrap 前：远端 `REMOTE_SRC_DIR` 是初始可信来源。
- bootstrap 后：WSL 本地项目目录是源码唯一可信来源。
- `REMOTE_SRC_DIR`：远端源码镜像目录，只由 rsync 写入；ROS2 通常为 `/home/orangepi/ros2_ws/src`。
- `REMOTE_WORK_DIR`：远端执行构建/运行命令的工作目录；ROS2 通常为 `/home/orangepi/ros2_ws`。
- 构建产物应尽量放在源码目录外；如果必须放在源码目录内，应确保对应目录被 rsync exclude 排除。

## 默认排除项

同步默认排除：

- `.git/`
- `.claude/`
- `scripts/remote.env`
- `build/`, `install/`, `log/`, `dist/`, `out/`, `target/`
- `.cache/`, `.pytest_cache/`, `__pycache__/`
- `node_modules/`

如需额外排除，在 `scripts/remote.env` 中设置：

```sh
RSYNC_EXTRA_EXCLUDES=".vscode/ large-data/"
```

## 安全注意事项

- 不要把 `REMOTE_SRC_DIR` 设置为 `/`、`~`、`/home/root`、`/home/<user>` 或 ROS2 工作区根目录等宽泛目录。
- 第一次远端到本地拉取不使用 `--delete`，避免删除本地模板脚本和文档。
- 第一次本地到远端同步必须先 dry run。
- `rsync --delete` 只应该作用在专门给这个项目准备的远端源码镜像目录。
- 推荐使用 SSH key 登录开发板。
- 最终可信验证以远端 `scripts/remote-build.sh` 和 `scripts/remote-run.sh` 为准，本地 WSL 检查只能作为辅助。
