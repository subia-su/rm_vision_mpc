# 发给队员：完整上位机源码与编译

交付版本：`v2026.11.27-source.1`。源码位于本队仓库 `xkddyl/rm_vision_mpc`，包含世界Pitch接口修复和离线坐标诊断。本包供Ubuntu编译验收，尚未完成本版本的ROS2完整构建和实车验收。

## 获取固定版本

从 [GitHub发布页](https://github.com/xkddyl/rm_vision_mpc/releases/tag/v2026.11.27-source.1) 下载 `rm_vision_mpc-v2026.11.27-source.1.zip`。解压后进入 `rm_vision_mpc`，应直接看到 `src`、`tools`、`README.md`、本文档。不要只取 `rm_serial_driver` 子目录，也不要覆盖旧工程的 `build/install/log`。

也可通过Git取得同一份源码：

```bash
git clone --branch v2026.11.27-source.1 --depth 1 https://github.com/xkddyl/rm_vision_mpc.git
cd rm_vision_mpc
```

包内包含全部已跟踪的源码、配置、模型、海康SDK及测试文档；系统ROS、OpenVINO、本地OpenCV安装、训练数据集和旧编译产物不随包分发。发布页附有逐文件清单和SHA256校验，ZIP与发布标签对应同一提交。

## 先编译通信接口

目标环境为Ubuntu 22.04 + ROS2 Humble；先安装该ROS发行版。以下命令只安装依赖、编译和运行主机测试，不启动节点，不打开串口或驱动电机。

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  libasio-dev libeigen3-dev

# 首次使用rosdep才执行sudo rosdep init；已经初始化则跳过。
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/transport_drivers src/rm_serial_driver \
  src/rm_auto_aim/auto_aim_interfaces --ignore-src --rosdistro humble -r -y

source tools/setup_local.sh
colcon build --symlink-install --packages-up-to rm_serial_driver \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rm_serial_driver --event-handlers console_direct+
colcon test-result --verbose
```

这一阶段只验证消息接口、串口驱动及其依赖，不需要相机、C板、CUDA或OpenVINO。单独的协议回归也可执行：

```bash
python3 tools/test_world_contract.py
```

不要把“通信包编译通过”当作完整视觉工程已通过。若命令失败，先保存第一处错误和 `colcon test-result --verbose`，不要跳过失败或恢复旧Pitch限位来使测试通过。

## 再编译完整视觉工程

先按 [README的环境章节](README.md#环境) 安装ROS图像、相机、TF等依赖。CPU YOLO26路径当前要求OpenVINO 2026.4.1和独立安装的OpenCV 4.12；这是工程现有依赖版本，本次打包没有更换依赖或求解器。已有相同依赖可复用；新电脑执行仓库现有安装脚本：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
bash tools/install_openvino.sh
bash tools/build_opencv_dnn.sh

source tools/setup_local.sh
test -f "$RM_VISION_OPENVINO_DIR/runtime/cmake/OpenVINOConfig.cmake"
test -f "$RM_VISION_OPENCV_DNN_DIR/lib/cmake/opencv4/OpenCVConfig.cmake"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

安装脚本需要联网下载第三方依赖。`tools/setup_local.sh` 按解压目录计算路径，加载工作区上一级 `.deps`；不要把原电脑的 `/home/rm/...` 写入新电脑配置。`OpenVINOConfig.cmake` 或 `OpenCVConfig.cmake` 缺失时，先处理相应依赖，不能把本地OpenCV路径改成系统4.5来绕过CPU检测器的4.12要求。

完整工程保留传统CV、TensorRT可选路径和CPU YOLO26路径。TensorRT/CUDA在传统检测包中可选；直接构建全部包仍会构建需要OpenVINO的 `yolo26_dnn_detector`。海康SDK包含amd64和arm64库，但本包内的OpenVINO安装脚本针对Ubuntu22 x86_64；Jetson/arm64不能直接套用该安装脚本。

本次打包没有运行上述Ubuntu完整构建；请队员实际编译后回传结果。Windows上的协议测试通过不等同于ROS节点已经编译或运行。

## 联调之前必须核对

- A6为38字节；世界/水平Yaw、Pitch单位rad，角速度rad/s，角加速度rad/s²。现有A6/5A格式和CRC保持不变。
- 使用世界Pitch配套下位机：`C_MPC_PITCH_WORLD_INPUT=1`，A6固定TTL为40ms。删除旧的 `mpc.pitch_min/max` 参数，不能把世界Pitch重新截到DM机械区间。详见 [世界Pitch契约](docs/world_pitch_contract_20261127.md)。
- 上位机目前实际调用项目自带 `MpcPlanner`，本次没有替换成TinyMPC库。下位机保留反馈控制，不运行TinyMPC。
- Yaw原点/符号仍待机械确认；主机已复现零角对应180°控制误差。新包不能直接视为可执行自瞄运动的验收版本。静态记录项目见 [坐标诊断表](docs/offline_coordinate_diagnostics.md)。
- 相机内参/外参、敌方颜色、USB设备路径由现场核对；现有5A不是电机角速度或执行ACK。编译阶段不需要插板，不启动完整bringup来代替编译测试。

本次没有修改PID、机械零位、齿比、发射逻辑或下位机固件，没有烧录。发布页保存本次PC测试范围与未验收项。
