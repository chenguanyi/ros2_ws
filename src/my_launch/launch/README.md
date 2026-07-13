# RKNN YOLO 实时摄像头显示

在 Orange Pi 的**本地图形桌面终端**运行以下命令，会启动 USB 摄像头、RKNN YOLO 检测与 `rqt_image_view`，并在屏幕显示绿色检测框：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch my_launch rknn_yolo_camera.launch.py \
  model_path:=/home/orangepi/models/h2025_yolov5s_fp.rknn \
  classes_path:=/home/orangepi/models/classes.txt \
  camera_device:=/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0 \
  fourcc:=MJPG \
  display:=true
```

- `display:=true` 只应在连接显示器的 Orange Pi 本地图形桌面中使用；普通 SSH 会话没有图形环境，launch 会跳过 GUI，但相机和 YOLO 会继续运行。
- `fourcc:=MJPG` 需要摄像头支持。先用 `v4l2-ctl -d <设备路径> --list-formats-ext` 确认；若无法打开摄像头，移除该参数以使用驱动默认格式。
- 显示窗口订阅 `/yolo/debug_image`。`publish_debug_image` 默认为 `true`；若将它设为 `false`，则不能同时使用 `display:=true`。

## Headless / 性能模式

在 SSH、systemd 或不需要预览时运行：

```bash
ros2 launch my_launch rknn_yolo_camera.launch.py \
  model_path:=/home/orangepi/models/h2025_yolov5s_fp.rknn \
  classes_path:=/home/orangepi/models/classes.txt \
  display:=false \
  publish_debug_image:=false
```

## 排障

确认相机、带框图像和检测结果话题都存在：

```bash
ros2 topic list | grep -E '/camera/image_raw|/yolo/debug_image|/yolo/detections'
ros2 topic hz /yolo/debug_image
```

确认当前终端具有图形会话：

```bash
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s\n' "$DISPLAY" "$WAYLAND_DISPLAY"
```

若提示缺少显示工具，在板端安装：

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rqt-image-view
```
