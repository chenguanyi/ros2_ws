---
description: "一键 dry-run 检查并推送本地 ROS2 源码到远端 Orange Pi。"
---

当用户输入 `/push-remote` 时，执行当前工作区脚本：

```bash
/home/ubuntu/remote-board-dev/scripts/push-remote.sh
```

## 行为

- 先运行 `scripts/rsync-to-remote.sh --dry-run`，展示即将同步/删除的变化。
- 再运行 `scripts/rsync-to-remote.sh`，把本地源码推送到远端 `REMOTE_SRC_DIR`。
- 该同步脚本使用 `--delete`，但依赖 `scripts/remote.env` 中的 `LOCAL_SOURCE_IS_CANONICAL=1` 保护。
- 不自动构建、不自动运行；构建仍用 `/build-remote` 或 `scripts/remote-build.sh`（如果之后创建）。

## 输出要求

用中文简短汇报结果：

- dry-run 是否成功；
- 实际推送是否成功；
- 如果失败，摘录关键错误并说明需要用户处理的配置或网络问题。
