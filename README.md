# object_tracking



## 项目简介

ROS2 目标追踪节点，支持 ByteTrack 和 OC-SORT 两种追踪算法，基于 `vision_service.h`（`model_zoo/vision`）实现多目标持续跟踪。

## 功能特性

- 支持 ByteTrack 和 OC-SORT 追踪算法
- 可配置切换追踪器
- 输出持续的 track_id
- 提供视频测试工具
- 不支持：跨摄像头追踪

## 快速开始

### 环境准备

- ROS2 Humble 或更高版本
- 已编译的 `components/model_zoo/vision` 组件
- 对应的模型文件

### 构建编译

```bash
colcon build --packages-select object_tracking
source install/setup.bash
```

### 运行示例

```bash
ros2 launch object_tracking object_tracking.launch.py
```

## 详细使用

### 依赖

- `components/model_zoo/vision`：提供检测/追踪所需运行库
- 对应追踪配置文件：`config/bytetrack.yaml` 或 `config/ocsort.yaml`

### 话题

| 类型 | 话题（默认） | 说明 |
|------|--------------|------|
| 订阅 | `/camera/image_raw` | 输入图像 |
| 发布 | `/perception/tracks` | vision_msgs/Detection2DArray（含 track_id） |
| 发布 | `/object_tracking/boxes` | Float32MultiArray，每目标 7 个数：x1,y1,x2,y2,score,label,**track_id** |
| 发布 | `/object_tracking/debug_image` | 带追踪框的可视化图 |

### 配置

主要配置文件：`config/object_tracking.yaml`

- **tracker_type**：`"bytetrack"` 或 `"ocsort"`。当 `config_path` 为空时，自动加载 `config/<tracker_type>.yaml`。
- **config_path**：留空则按 `tracker_type` 使用包内 `config/bytetrack.yaml` 或 `config/ocsort.yaml`；也可指定其它 yaml 路径。
- **score_threshold**、**image_topic**、**boxes_topic**、**debug_image_topic**、**detections_topic** 等与其它 perception 节点一致。


默认使用 ByteTrack。改用 OC-SORT 时，在 `object_tracking.yaml` 中设置：

```yaml
tracker_type: "ocsort"
```

或直接指定 config_path 指向自定义 yaml。

## 常见问题

### 无相机时用视频测试

包内提供脚本，将本地视频按帧发布到 `/camera/image_raw`：

```bash
# 终端 1：发布视频（默认按视频 fps，播完循环）
ros2 run object_tracking publish_video /path/to/video.mp4

# 指定发布频率 15 Hz，播完不循环
ros2 run object_tracking publish_video /path/to/video.mp4 --rate 15 --no-loop

# 终端 2：启动追踪节点
ros2 launch object_tracking object_tracking.launch.py
```

依赖：`pip install opencv-python-headless`（读视频）。

### 简单图像显示（无 rqt/rviz 时）

用 OpenCV 窗口显示图像话题（需本机有显示或 DISPLAY）：

```bash
# 默认显示 /object_tracking/debug_image（带追踪框的图）
ros2 run object_tracking simple_image_viewer

# 指定其他图像话题
ros2 run object_tracking simple_image_viewer /camera/image_raw
ros2 run object_tracking simple_image_viewer /human_detection/debug_image
```

说明：`/perception/tracks` 是 Detection2DArray（框数据），不能直接显示；要“看”追踪效果请订阅上面的图像话题（如 `/object_tracking/debug_image`）。

## 版本与发布

当前版本：1.0.0

变更记录：
- 初始版本发布

## 贡献方式

欢迎提交 Issue 和 Pull Request。

贡献者与维护者名单见：`CONTRIBUTORS.md`（如有）

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
