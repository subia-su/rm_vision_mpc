# rm_vision

RoboMaster 装甲板视觉系统，运行于 Ubuntu 22.04 + ROS 2 Humble。
当前版本同时保留传统 CV、TensorRT 和 OpenVINO CPU 三条检测路径。
本机实时路径为：

```text
Hikrobot 640×640
    ├── YOLO26-OBB Accuracy Control INT8
    ├── OpenVINO 2026.4.1 CPU
    └── 相机/检测器 composable 零拷贝
```

本地仓库：

```text
/home/rm/rm_vision/rm_vision
```

## 当前验证状态

| 项目 | 实测结果 |
|---|---|
| 相机 | Hikrobot MV-CS016-10UC |
| 相机输出 | 640×640，46 FPS |
| 中心 ROI | offset `(400, 220)` |
| `/detector/armors` | 46.0 FPS |
| 视觉端延迟 | 约 15.7 ms |
| Int8 模型 P50 | 11.07 ms |
| Int8 模型 P95 | 11.50 ms |
| 视觉容器 CPU | 约 2.4 个逻辑核 |
| 串口反馈 | 约 100 Hz，CRC errors=0 |
| 云台 TF | 约 100 Hz |

当前实车画面没有装甲板目标，因此检测、串口、TF 和跟踪节点频率均已验证，
但真实目标下的 `tracking=true` 和云台控制闭环仍需复验。

新增回归测试共 9 项，覆盖 PnP 几何/时序门控、长短边交换角度等价、
640 快速预处理、置信度噪声缩放和 EKF 更新增益。全部通过。

## 数据流

```text
Hikrobot USB Camera
        │ /image_raw (rgb8)
        ▼
hik_camera ──┐
             │ 同一 composable container
             │ intra-process zero-copy
             ▼
yolo26_dnn_detector
        │ /detector/armors
        │ /detector/yolo_boxes
        ▼
armor_tracker
        │ /tracker/target
        ▼
rm_serial_driver ──USB CDC──► STM32 电控
        ▲
        └──云台 roll/pitch/yaw 反馈
```

TF 树：

```text
odom
  └── gimbal_link
        └── camera_link
              └── camera_optical_frame
```

视觉端输出检测、PnP 和 EKF 目标状态。弹道解算、云台闭环和电机控制由
STM32 端完成。

## 核心逻辑

### 相机 ROI

海康传感器全幅为 1440×1080。启动时先恢复全幅，再设置中心 640×640：

```text
offset_x = (1440 - 640) / 2 = 400
offset_y = (1080 - 640) / 2 = 220
```

驱动会校验 `Width`、`Height`、`OffsetX`、`OffsetY` 和 `AcquisitionFrameRate`。
如果相机保留了上一次 ROI，驱动会重置 Offset、恢复全幅，再重新裁剪。

### 图像转换

海康 SDK 图像转换为 RGB8 前，驱动先分配目标缓冲区并设置
`nDstBufferSize`、`pDstBuffer`，之后才调用 `MV_CC_ConvertPixelType()`。
转换失败时丢弃当前帧，不发布未初始化图像。

### YOLO26-OBB Int8

模型协议：

```text
输入:  [1, 3, 640, 640] RGB FP32，数值范围 [0, 1]
输出:  [1, 41, 8400]
特征:  cx, cy, width, height, 36 class scores, angle
```

量化流程：

1. 将 ONNX 转换为固定输入 OpenVINO FP32 IR。
2. 从训练集按类别分层抽样 300 张图像做 PTQ 校准。
3. 使用 NNCF `MIXED` preset、完整 bias correction 和 CPU target。
4. 用 200 张验证图执行 Accuracy Control，最大允许 mAP 下降 0.01。
5. 首层卷积 `/model.0/conv/Conv` 自动回退为 FP32。
6. 保存 IR 时设置 `compress_to_fp16=False`。

项目使用直接 OpenVINO C++ 推理。OpenCV 负责预处理、OBB 解码、分类别 NMS
和 PnP。这样避免旧 OpenCV Inference Engine 包装层的性能损失。

640×640、连续内存的 RGB 图像会跳过 resize 和 letterbox，直接生成 DNN blob。
非 640 输入仍自动执行 letterbox。

### PnP 质量控制

每块装甲板在进入跟踪器前经过：

- 相机前方深度检查
- 装甲板长宽比检查
- 四角点重投影误差检查
- 短时间内的 3D 位置跳变检查
- 图像中心像素跳变检查
- 按装甲板长轴归一化后的周期角度检查

长轴角度使用模 `π` 表示，并处理 width/height 互换导致的 90° 表示差异。
异常结果不会更新连续帧状态，下一次合法检测可以直接恢复。

### 进程内通信

相机和 Int8 检测器注册为 composable node，在同一个
`component_container` 中启用 `use_intra_process_comms=True`。

独立进程通过 DDS 接收 640×640 原始图像时约 38–40 FPS；
改为进程内通信后达到相机上限 46 FPS。

### 颜色与协议编号

线上检测通过 `detect_color` 只保留一个阵营：

```text
0 = 红方
1 = 蓝方
```

模型角色映射到串口协议编号：

| 模型角色 | 协议编号 |
|---|---|
| outpost | `outpost` |
| sentry | `guard` |
| hero | `1` |
| engineer | `2` |
| infantry_3/4/5 | `3/4/5` |
| base_small/base_large | `base` |

gray 和 purple 类别主要用于数据集验证，不进入线上红蓝过滤。

## ROS 包

工作区包含 14 个 ROS 2 包：

| 包 | 职责 |
|---|---|
| `hik_camera` | 海康相机采集和 CameraInfo |
| `yolo26_dnn_detector` | OpenVINO FP32/Int8 和 OpenCV DNN 后端 |
| `armor_detector` | 传统 CV、TensorRT YOLO、PnP |
| `armor_tracker` | EKF 目标跟踪 |
| `auto_aim_interfaces` | Armor、Target、YoloBox 等消息 |
| `rm_serial_driver` | 视觉与电控串口协议 |
| `serial_driver` | 串口底层封装 |
| `io_context` | ASIO I/O 上下文 |
| `asio_cmake_module` | 独立 ASIO CMake 模块 |
| `udp_driver` | 可选 UDP 传输 |
| `udp_msgs` | UDP 消息 |
| `rm_gimbal_description` | 云台 URDF |
| `rm_vision_bringup` | 参数和 launch |
| `rm_auto_aim` | 检测与跟踪元包 |

## 目录结构

```text
rm_vision/
├── README.md
├── datasets/
│   └── yolo26_obb/
│       ├── dataset.yaml
│       ├── classes.txt
│       ├── images/{train,val}/
│       └── labels/{train,val}/
├── experiments/
│   └── yolo26_onnx_cpu/
│       ├── cpp/
│       ├── quantize_openvino.py
│       ├── quantize_accuracy_control.py
│       ├── validate_opencv_dnn.py
│       ├── validate_openvino.py
│       └── results/
├── src/
│   ├── rm_auto_aim/
│   ├── rm_gimbal_description/
│   ├── rm_serial_driver/
│   ├── rm_vision_bringup/
│   ├── ros2_hik_camera/
│   └── transport_drivers/
└── tools/
    ├── setup_local.sh
    ├── install_openvino.sh
    ├── build_opencv_dnn.sh
    ├── auto_start.sh
    ├── rm_vision.service
    ├── camera_monitor.py
    └── armor_visualizer.py
```

以下内容不进入 Git：

```text
build/
install/
log/
.ros/
logs/
datasets/yolo26_obb/images/
datasets/yolo26_obb/labels/
experiments/yolo26_onnx_cpu/.venv/
```

## 数据和模型

数据集：

```text
train:   5497
val:      607
total:   6104
classes:    36
format:   YOLO OBB
```

模型文件：

```text
src/rm_auto_aim/armor_detector/model/
├── yolo26_obb.onnx
├── yolo26_obb_fp32.xml
├── yolo26_obb_fp32.bin
├── yolo26_obb_int8.xml
├── yolo26_obb_int8.bin
└── yolo26_classes.txt
```

正式 `yolo26_obb_int8.xml/.bin` 是 Accuracy Control 版本。

## 环境

已验证：

```text
Ubuntu 22.04.5 LTS
ROS 2 Humble
Python 3.10
GCC 11
CMake 3.22
Intel Core i7-13620H
```

ROS 依赖：

```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager \
  ros-humble-camera-calibration \
  ros-humble-vision-opencv \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-xacro \
  ros-humble-ament-cmake-clang-format \
  libasio-dev \
  libeigen3-dev \
  python3-pip \
  python3.10-venv
```

OpenVINO 和 OpenCV 安装在用户依赖目录，不覆盖系统 ROS：

```bash
cd /home/rm/rm_vision/rm_vision
bash tools/install_openvino.sh
bash tools/build_opencv_dnn.sh
```

安装结果：

```text
/home/rm/rm_vision/.deps/openvino-2026.4.1
/home/rm/rm_vision/.deps/opencv-4.12-openvino
```

系统 ROS 仍使用 OpenCV 4.5.4；YOLO26 Int8 节点使用本地 OpenCV 4.12
和 OpenVINO 2026.4.1。

Python 实验环境：

```bash
cd /home/rm/rm_vision/rm_vision/experiments/yolo26_onnx_cpu
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 串口权限

```bash
sudo usermod -aG dialout "$USER"
```

注销并重新登录后生效。

当前设备：

```text
/dev/ttyACM0
```

稳定路径：

```text
/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_207037765948-if00
```

## 构建

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source tools/setup_local.sh
```

`setup_local.sh` 会加载：

- ROS 2 Humble
- OpenVINO
- 当前工作区安装环境
- 工作区内 ROS 日志目录
- 本地 OpenCV DNN 路径

## 运行

### YOLO26 OpenVINO Int8 实时管线

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
ros2 launch rm_vision_bringup yolo26_cpu_bringup.launch.py
```

启动：

```text
robot_state_publisher
camera_detector_container
  ├── hik_camera
  └── yolo26_dnn_detector
rm_serial_driver
armor_tracker
```

### 传统 CV / TensorRT 管线

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

该入口使用 `armor_detector`：

- `use_yolo: false`：传统 CV
- `use_yolo: true`：TensorRT YOLO

### 无硬件模式

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
ros2 launch rm_vision_bringup no_hardware.launch.py
```

### 红蓝方切换

```bash
ros2 param set /armor_detector detect_color 0   # 红方
ros2 param set /armor_detector detect_color 1   # 蓝方
```

## 参数

主文件：

```text
src/rm_vision_bringup/config/node_params.yaml
```

### 相机

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `exposure_time` | 6000 | 微秒 |
| `gain` | 8.0 | 增益 |
| `frame_rate` | 46.0 | 目标帧率 |
| `width` / `height` | 640 / 640 | 输出 ROI |
| `sensor_width` / `sensor_height` | 1440 / 1080 | 传感器全幅 |
| `offset_x` / `offset_y` | -1 / -1 | -1 表示自动居中 |

实际中心偏移应为：

```text
(400, 220)
```

### 串口

| 参数 | 当前值 |
|---|---:|
| `device_name` | `/dev/ttyACM0` |
| `baud_rate` | 115200 |
| `flow_control` | none |
| `parity` | none |
| `stop_bits` | 1 |
| `timestamp_offset` | 0.006 s |

### OpenVINO Int8 检测器

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `dnn.backend` | `openvino_int8` | 也支持 `openvino_fp32`、`opencv_dnn` |
| `dnn.conf_threshold` | 0.35 | 部署阈值 |
| `dnn.nms_threshold` | 0.5 | 分类别 NMS |
| `dnn.threads` | 6 | P-core 线程数 |

### PnP 过滤

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `filter.max_reprojection_error_px` | 8.0 | 四角点最大重投影误差 |
| `filter.min_box_aspect` | 1.2 | OBB 长边/短边下限 |
| `filter.max_box_aspect` | 6.5 | OBB 长边/短边上限 |
| `filter.min_depth_m` | 0.05 | 相机前方最近距离 |
| `filter.max_depth_m` | 12.0 | 最大相机深度 |
| `filter.temporal_gate_seconds` | 0.15 | 连续帧门控窗口 |
| `filter.max_position_jump_m` | 1.5 | 窗口内最大 3D 跳变 |
| `filter.max_image_jump_px` | 220.0 | 窗口内最大像素跳变 |
| `filter.max_angle_jump_rad` | 1.4 | 窗口内最大长轴角度跳变 |

### 跟踪器

| 参数 | 当前值 |
|---|---:|
| `target_frame` | `odom` |
| `max_armor_distance` | 10.0 m |
| `ekf.sigma2_q_xyz` | 0.05 |
| `ekf.sigma2_q_yaw` | 5.0 |
| `ekf.sigma2_q_r` | 80.0 |
| `ekf.r_xyz_factor` | 0.0004 |
| `ekf.r_yaw` | 0.005 |
| `ekf.confidence_reference` | 0.7 |
| `ekf.confidence_r_min_scale` | 0.25 |
| `ekf.confidence_r_max_scale` | 4.0 |
| `tracker.max_match_distance` | 1.0 m |
| `tracker.max_match_yaw_diff` | 2.0 rad |
| `tracker.tracking_thres` | 5 |
| `tracker.lost_time_thres` | 1.0 s |

Armor 消息携带检测置信度和重投影误差。EKF 测量噪声按
`(reference / confidence)^2` 缩放并限幅；置信度越高，测量权重越大。
置信度缺失或小于等于 0 时退回中性倍率 1.0，兼容传统检测器。

## 精度和性能

完整验证集为 607 张。

### 精度

| 模型/阈值 | mAP@0.5 | Precision | Recall | F1 |
|---|---:|---:|---:|---:|
| FP32, 0.25 | 0.5995 | 0.8257 | 0.8962 | 0.8595 |
| Int8 基础 PTQ, 0.25 | 0.5854 | 0.7099 | 0.8898 | 0.7897 |
| Int8 Accuracy Control, 0.25 | 0.5936 | 0.7126 | 0.8926 | 0.7925 |
| Int8 Accuracy Control, 0.35 | 0.5806 | 0.7381 | 0.8797 | 0.8027 |

Accuracy Control 将基础 PTQ 的 mAP 降幅从 0.0141 改善到 0.0059。

### 推理

| 后端 | 平均推理 | P95 |
|---|---:|---:|
| OpenCV CPU FP32 | 48.74 ms | 52.34 ms |
| OpenVINO FP32 | 13.89 ms | 14.54 ms |
| OpenVINO Int8 基础 PTQ | 6.86 ms | 7.74 ms |
| OpenVINO Int8 Accuracy Control | 7.58 ms | 8.59 ms |
| 直接 OpenVINO C++ Int8，6 线程 | 11.07 ms | 11.50 ms |

### 真机管线

```text
/image_raw:        46 FPS
/detector/armors:  46 FPS
端到端延迟:      约 15.7 ms
视觉容器 CPU:    约 2.4 核
云台 TF:          100 Hz
```

性能文件：

```text
experiments/yolo26_onnx_cpu/results/
```

## Int8 复现

```bash
cd /home/rm/rm_vision/rm_vision
source ../.deps/openvino-2026.4.1/setupvars.sh
cd experiments/yolo26_onnx_cpu

.venv/bin/python quantize_openvino.py \
  --calibration-size 300 \
  --preset mixed

.venv/bin/python quantize_accuracy_control.py \
  --calibration-size 300 \
  --validation-size 200 \
  --max-drop 0.01

.venv/bin/python validate_openvino.py \
  --model ../../src/rm_auto_aim/armor_detector/model/yolo26_obb_int8.xml \
  --output results/openvino_int8_final
```

## ROS 接口

### 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | 640×640 RGB8 |
| `/camera_info` | `sensor_msgs/CameraInfo` | 640×640 内参 |
| `/detector/armors` | `auto_aim_interfaces/Armors` | PnP 后装甲板 |
| `/detector/yolo_boxes` | `auto_aim_interfaces/YoloBoxes` | OBB 框数据 |
| `/tracker/target` | `auto_aim_interfaces/Target` | EKF 目标 |
| `/tracker/info` | `auto_aim_interfaces/TrackerInfo` | 跟踪残差 |
| `/latency` | `std_msgs/Float64` | 视觉到串口延迟 |
| `/tf` | `tf2_msgs/TFMessage` | 云台和相机 TF |
| `/aiming_point` | `visualization_msgs/Marker` | 瞄准点 |

### 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `/tracker/reset` | `std_srvs/Trigger` | 重置跟踪器 |
| `/armor_detector/set_parameters` | `rcl_interfaces/SetParameters` | 修改检测参数 |

### 诊断命令

```bash
source tools/setup_local.sh
ros2 node list
ros2 topic hz /image_raw
ros2 topic hz /detector/armors
ros2 topic hz /tracker/target
ros2 topic echo /detector/armors --once
ros2 topic echo /tracker/target --once
ros2 topic echo /latency --once
ros2 run tf2_ros tf2_echo odom gimbal_link
```

`setup_local.sh` 默认设置 `ROS_LOCALHOST_ONLY=1`。

## 串口协议

实现：

```text
src/rm_serial_driver/include/rm_serial_driver/packet.hpp
```

配置：115200、8N1、无流控。

所有浮点字段按 `int16 ×100` 小端传输。

| 方向 | 帧头 | 长度 | 内容 |
|---|---:|---:|---|
| 电控 → 视觉 | `0x5A` | 16 B | flags + 6×int16 + CRC |
| 视觉 → 电控 | `0xA5` | 26 B | flags + 11×int16 + CRC |

CRC：

```text
reflected polynomial: 0x8408
initial value:        0xFFFF
final xor:            none
```

上行：

```text
roll, pitch, yaw, aim_x, aim_y, aim_z
```

下行：

```text
x, y, z, yaw, vx, vy, vz, v_yaw, r1, r2, dz
```

下行内容是 EKF 后的目标状态，不是云台角度或弹道解算结果。

驱动会处理拆包、合包、短写和 CRC 错误，并持续写完完整 26 字节下行帧。

## 相机标定

内参：

```text
src/rm_vision_bringup/config/camera_info.yaml
```

约束：

- 图像宽高必须和 CameraInfo 一致
- 改变 ROI 尺寸或中心后必须重新换算主点或重新标定
- 当前标定目标是 640×640

标定命令：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x7 \
  --square 0.04 \
  --pattern chessboard \
  --ros-args \
  -r image:=/image_raw \
  -r camera:=/camera_info
```

建议采集：

- 40–60 个有效姿态
- 画面中心与四角
- 多个距离和三维倾角
- 重投影误差低于 0.5 px

外参：

```text
src/rm_vision_bringup/config/launch_params.yaml
```

当前值：

```text
odom2camera:
  xyz: "0.10 0.0 0.05"
  rpy: "0.0 0.0 0.0"
```

必须根据相机相对云台转轴的实际安装位置和方向修改。

验证：

```bash
ros2 run tf2_ros tf2_echo odom camera_optical_frame
```

## 可视化

相机监控：

```bash
source tools/setup_local.sh
python3 tools/camera_monitor.py --port 8080
```

OBB 可视化：

```bash
source tools/setup_local.sh
python3 tools/armor_visualizer.py --port 8081
```

网页：

```text
http://127.0.0.1:8080/
http://127.0.0.1:8081/
```

## systemd

安装：

```bash
sudo cp tools/rm_vision.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rm_vision
```

管理：

```bash
systemctl status rm_vision
sudo systemctl restart rm_vision
sudo systemctl stop rm_vision
sudo systemctl disable rm_vision
```

注意：当前 `auto_start.sh` 启动的是传统
`vision_bringup.launch.py`，不是 YOLO26 Int8 入口。如需开机默认运行 Int8，
先修改脚本中的 launch 文件，并完成目标闭环验证。

日志：

```text
logs/vision_bringup.log
logs/visualizer.log
logs/ros/
```

## 问题排查

### 相机不是 640×640

```bash
lsusb
ros2 topic echo /image_raw --once --field width
ros2 topic echo /image_raw --once --field height
ros2 topic echo /camera_info --once --field width
```

预期均为 640。

启动日志应包含：

```text
ROI: 640x640 offset=(400,220)
```

### 串口无法打开

```bash
ls -l /dev/ttyACM0
groups
```

确保当前会话包含 `dialout`，加入组后重新登录。

### 没有 odom→gimbal_link

```bash
ros2 run tf2_ros tf2_echo odom gimbal_link
```

该 TF 依赖合法电控反馈。检查串口路径、协议帧和 CRC。

### 检测频率低于 46 FPS

```bash
ros2 param get /armor_detector dnn.backend
ros2 param get /armor_detector dnn.threads
ros2 topic hz /image_raw
ros2 topic hz /detector/armors
```

预期：

```text
openvino_int8
6
46 Hz
```

如果相机和检测器不在同一个 composable container，原始图像会经过 DDS，
通常只有 38–40 FPS。

### DDS 状态异常

```bash
export ROS_LOCALHOST_ONLY=1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

然后重启全部 ROS 进程。

## 已知限制

- 尚未在真实装甲板目标前完成 `tracking=true` 和电控闭环验证。
- 36 类数据分布不均衡，部分前哨站、基地和 baseline 类别验证样本极少。
- 当前现场光照和 6000 μs 曝光下画面偏暗，远距离召回不足时先调曝光和增益。
- TensorRT 模式需要 CUDA、TensorRT 和匹配设备的 engine。
- systemd 默认仍使用传统 CV 入口。
- Int8 最终阈值 0.35 的 mAP 是 0.5806，低于 FP32 的 0.5995；
  选择 0.35 是为了降低线上假阳性并提高 F1。

## Git

远端：

```text
https://github.com/subia-su/rm_vision.git
```

查看：

```bash
git status
git log --oneline --decorate -5
```

## 致谢

项目基于 RoboMaster 视觉社区和以下上游工作继续开发：

- chenjunnn/rm_vision
- PennState-RoboX/rm_vision
- Hikrobot MVS SDK
- ROS 2、OpenCV、OpenVINO、NNCF

仅用于 RoboMaster 学习、研发与比赛调试。
