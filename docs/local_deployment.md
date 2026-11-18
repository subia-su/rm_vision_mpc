# 本机部署说明（Ubuntu 22.04 + ROS 2 Humble）

本文记录当前开发机上的实际部署方式。上游项目原始目标平台是 Jetson +
Ubuntu 20.04 + ROS 2 Foxy，本机已调整为 Ubuntu 22.04 + ROS 2 Humble
的 x86_64 CPU 环境。

## 1. 部署位置

项目源码与本地 Git 仓库位于：

```text
/home/rm/rm_vision/rm_vision
```

工作区上一级的 `/home/rm/rm_vision/.git` 是运行环境挂载的只读占位点，
不能作为项目仓库，因此项目仓库放在子目录 `rm_vision/` 中。

## 2. 本机环境

| 项目 | 当前环境 | 说明 |
|---|---|---|
| 系统 | Ubuntu 22.04.5 LTS x86_64 | 与原 Jetson 环境不同 |
| ROS | ROS 2 Humble | 原项目为 ROS 2 Foxy |
| Python | 3.10.12 | 使用系统 Python |
| 编译器 | GCC / CMake 3.22 | 已完成全量编译 |
| GPU | 未检测到 NVIDIA CUDA/TensorRT | 默认使用传统 CV 检测器 |
| 相机 | Hikrobot `2bdf:0001`，SDK 直接访问 USB | 已通过 640×640 ROI 验证 |
| 串口 | STM32 CDC `/dev/ttyACM0`，属于 `dialout` | 已通过 100 Hz 反馈验证 |

## 3. 安装依赖

本机已安装以下 ROS 2 Humble 与系统开发依赖：

```bash
sudo apt install -y \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-camera-info-manager \
  ros-humble-camera-calibration ros-humble-vision-opencv \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-xacro \
  ros-humble-ament-cmake-clang-format \
  libasio-dev libeigen3-dev
```

如 `rosdep` 尚未初始化：

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

本机网络访问 rosdep 索引时曾出现超时；依赖是否齐全以本文的编译结果和
实际启动结果为准。

## 4. 编译

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source tools/setup_local.sh
```

本机验证结果：13 个包全部编译成功。`armor_detector` 已改为 TensorRT
可选；没有 TensorRT/CUDA 时自动使用传统 CV 检测器。

## 5. 无硬件启动

未连接相机和电控时，仅启动检测器、跟踪器与机器人状态发布器：

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
ros2 launch rm_vision_bringup no_hardware.launch.py
```

`tools/setup_local.sh` 会设置：

- `ROS_DISTRO=humble`
- `ROS_HOME=<工作区>/.ros`
- `ROS_LOG_DIR=<工作区>/logs/ros`
- `ROS_LOCALHOST_ONLY=1`（可通过预先设置环境变量覆盖）

## 6. 接入真机

确认设备：

```bash
ls -l /dev/ttyACM* /dev/video*
lsusb
```

将当前用户加入串口组并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

检查主参数 `src/rm_vision_bringup/config/node_params.yaml`：

- 串口 `device_name` 默认 `/dev/ttyACM0`
- 相机参数默认为海康中心 ROI 640×640、46 Hz；`offset_x/offset_y=-1`
  自动居中
- `use_yolo: false` 适合当前无 TensorRT 的开发机

连接硬件后启动完整管线：

```bash
cd /home/rm/rm_vision/rm_vision
source tools/setup_local.sh
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

当前版本只内置海康相机驱动，普通 USB 摄像头不能直接替代
`hik_camera`。相机内参、外参和 ROI 仍须按实际安装重新校验。

## 7. TensorRT / YOLO 模式

在具备 CUDA、TensorRT 和模型 engine 的 Jetson 上：

1. 将 `yolov8s_obb_fp16.engine` 放入
   `src/rm_auto_aim/armor_detector/model/`。
2. 将 `node_params.yaml` 中 `use_yolo` 改为 `true`。
3. 重新编译并启动。

如果编译时没有检测到 TensorRT/CUDA，即使误设 `use_yolo: true`，节点也会
打印警告并回退到传统 CV 检测器，不会因缺少 GPU 库直接退出。

## 8. 验证记录

已执行：

- 全量 `colcon build`：13 个包成功。
- `serial_packet_stream_test`：拆包、CRC 恢复、连续帧、短写、参考帧、
  flags 与数值边界测试通过。
- 海康相机硬件验证：实际输出 640×640（ROI 偏移 400,220）、46 Hz，
  `/image_raw` 与 `/camera_info` 分辨率一致，图像转换正常。
- 串口硬件验证：约 100 Hz 有效反馈、`CRC errors=0`，云台 TF 约 100 Hz。
- 当前画面没有装甲板目标，因此完整目标检测、跟踪和下行控制闭环仍需在
  实际目标场景中验证。

上游的 ROS 风格 lint 仍有既有告警，且 `xmllint` 需要访问在线 ROS schema；
这些不影响功能测试和本地运行。
