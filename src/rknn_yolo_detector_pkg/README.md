# rknn_yolo_detector_pkg

RK3588 NPU 加速 YOLOv5 ROS2 检测节点。节点订阅 `sensor_msgs/Image`，使用板端 `rknn-toolkit-lite2` 的 `RKNNLite` 加载 `.rknn` 模型，并发布任务节点容易解析的全量检测列表。

## NPU 调用方式

节点不会使用 CPU / PyTorch / OpenCV DNN fallback。启动时如果无法导入 `rknnlite.api`、无法加载 `.rknn`、或无法初始化 RKNN runtime，会直接报错退出。

为充分使用 RK3588 三个 NPU core，节点启动 3 个常驻 worker 线程：

| worker | RKNNLite 实例 | core mask |
| --- | --- | --- |
| 0 | 独立加载一次模型 | `RKNNLite.NPU_CORE_0` |
| 1 | 独立加载一次模型 | `RKNNLite.NPU_CORE_1` |
| 2 | 独立加载一次模型 | `RKNNLite.NPU_CORE_2` |

相机回调只把最新帧放入队列；三个 worker 并行取帧：

```text
线程 1：处理第 1 帧 -> NPU_CORE_0
线程 2：处理第 2 帧 -> NPU_CORE_1
线程 3：处理第 3 帧 -> NPU_CORE_2
```

队列满时丢弃旧帧，避免任务节点拿到严重滞后的检测结果。

## 板端依赖

在 Orange Pi / RK3588 板端安装 RKNN Lite runtime，例如官方 `rknn-toolkit-lite2` 对应版本。节点中实际调用：

```python
from rknnlite.api import RKNNLite
rknn.load_rknn(model_path)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)  # 每个 worker 分别绑定 0/1/2
outputs = rknn.inference(inputs=[input_tensor])
```

## 启动

先启动相机：

```bash
ros2 launch visual_pkg camera_source.launch.py
```

再启动 RKNN YOLO：

```bash
ros2 launch rknn_yolo_detector_pkg rknn_yolo_detector.launch.py \
  model_path:=/home/orangepi/models/yolov5.rknn \
  classes_path:=/home/orangepi/models/classes.txt
```

如果没有 `classes.txt`，需要显式传类别数：

```bash
ros2 launch rknn_yolo_detector_pkg rknn_yolo_detector.launch.py \
  model_path:=/home/orangepi/models/yolov5.rknn \
  num_classes:=3
```

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model_path` | 空 | `.rknn` 模型路径，必须传 |
| `classes_path` | 空 | 类别名文件，每行一个类别 |
| `num_classes` | `0` | 类别数；`0` 表示从 `classes_path` 行数读取 |
| `image_topic` | `/camera/image_raw` | 输入图像 |
| `detections_topic` | `/yolo/detections` | 检测结果 |
| `input_width` | `640` | 模型输入宽度 |
| `input_height` | `640` | 模型输入高度 |
| `input_color` | `rgb` | 模型输入颜色顺序，`rgb` 或 `bgr` |
| `conf_threshold` | `0.25` | 置信度阈值 |
| `nms_threshold` | `0.45` | NMS IoU 阈值 |
| `anchors` | YOLOv5s 默认 | 18 个数字，3 个尺度 x 3 个 anchor x 宽高 |
| `queue_size` | `3` | 三个 worker 共享待处理队列长度，满了丢旧帧 |
| `publish_debug_image` | `false` | 是否发布画框图像 |

## 输出格式

`/yolo/detections` 类型为 `std_msgs/msg/Float32MultiArray`。每个目标占 6 个 float：

```text
[class_id, confidence, x1, y1, x2, y2,
 class_id, confidence, x1, y1, x2, y2,
 ...]
```

坐标是原始图像像素坐标。

任务节点 C++ 解析示例：

```cpp
void onDetections(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  for (std::size_t i = 0; i + 5 < msg->data.size(); i += 6) {
    const int class_id = static_cast<int>(msg->data[i + 0]);
    const float conf = msg->data[i + 1];
    const float x1 = msg->data[i + 2];
    const float y1 = msg->data[i + 3];
    const float x2 = msg->data[i + 4];
    const float y2 = msg->data[i + 5];
  }
}
```

## 调试

打开画框图：

```bash
ros2 launch rknn_yolo_detector_pkg rknn_yolo_detector.launch.py \
  model_path:=/home/orangepi/models/yolov5.rknn \
  classes_path:=/home/orangepi/models/classes.txt \
  publish_debug_image:=true
```

运行时会节流打印每个 worker 的耗时：

```text
frame=123 worker=1 detections=2 preprocess=3.1ms inference=16.8ms postprocess=2.4ms dropped=0
```
