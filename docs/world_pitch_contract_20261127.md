# 世界Pitch对接与迁移（2026-11-27）

本地完整源码位于Windows `E:\RoboMaster\rm_vision_mpc`。此前从GitHub `subia-su/rm_vision_mpc` 提交 `35ca5d59cef9c6986cf994f0183be79b6c16b6e5` 下载完整归档，286个源文件逐个验证Git blob SHA；本地导入基线5eab0ce只用于最初的补丁审计。

现已将修复迁移到真实上游35ca5d5历史，并发布到本队主仓库 [xkddyl/rm_vision_mpc](https://github.com/xkddyl/rm_vision_mpc)。`origin`指向本队仓库，后续优先通过GitHub分支/提交链接协作；旧的本地导入历史没有推送。本文后面的补丁流程只用于核对现场旧版本，不替代当前GitHub主仓库。

## 改动

- `RMSerialDriver::sendData` 保留现有弹道预测和MpcPlanner优化器，将Pitch作为世界/水平角发送，不做DM区间硬截断，不额外反向。
- 新 `world_trajectory_contract.hpp` 检查坐标域、有限值、原速度/加速度上限及固定40ms时效。世界Pitch数学域是±π/2；这没有放宽DM机械限位。
- 规划异常、目标超时/丢失及契约不满足时发送更新序号的valid=0，清除无用浮点值；保留序号、host_ms、TTL和原38字节A6格式/CRC。不再只等下位机旧轨迹自然到期。
- A6年龄包含本次规划到发送前的steady-clock耗时（向上取整）和配置传输预算；它不重复加目标年龄。目标年龄已经用于预测，并另外受50ms数据源超时约束。host_ms仍不被当作MCU本地时钟。
- 单帧总年龄还预留MCU3ms CAN预算。它是接口预算，不代表已测得真实总线/USB延迟上界。MCU能识别其本地计时范围内的过期，但不能仅凭当前字段识别未计入的USB排队/运输延迟；该预算仍需实测验收。
- 新 `/serial_driver/planner_status`（std_msgs/String）报告最近发送帧的规划结果；`/serial_driver/trajectory`原11列不变。只读link_monitor追加最近序号、A6年龄、原因和原因接收年龄。原因与数组是两个ROS话题，不构成同步ACK；也不代表MCU已接纳。
- 优化器权重、弹道重力/弹速配置、符号、Yaw映射、USB/CAN布局、下位机PID与发射逻辑均未修改。本仓库实际调用自带MpcPlanner（投影梯度实现），本轮没有换用TinyMPC库；后续更换求解器仍可复用PlannedState和本契约。

## 配套下位机

使用 `infantry2.1` 的世界角分支：源提交cb67e5a、导出提交8103c81，配置 `C_MPC_PITCH_WORLD_INPUT=1`。该云台新固件目前尚未烧录，不可把此前N档恢复记录视为它已上车。

MCU用新鲜IMU Pitch与DM机械位置建立关系：`q_ref=q_measured-（pitch_world_ref-pitch_world_measured）`，机械范围仍为-0.39～+0.07rad。上位机世界角必须与既有5A报告的世界Pitch采用同一方向、水平零点及安装外参。

现有5A反馈没有DM机械位置、动态机械可达区间或命令ACK，因此上位机仅凭现有USB反馈不能保证目标机械可达。MCU是最终机械限位检查者，VOFA I46～I65提供拒绝/执行状态。Yaw π原点差和安装外参仍待确认，不因本补丁而消失。

## 参数迁移

删除旧的 `mpc.pitch_min`、`mpc.pitch_max`。发现这两个旧override时，本版会拒绝启动并解释原因，避免旧机械域配置悄悄限制世界角。

```yaml
trajectory.pitch_frame: world
trajectory.world_pitch_min: -1.5707963267948966
trajectory.world_pitch_max: 1.5707963267948966
trajectory.ttl_ms: 40
trajectory.transport_age_ms: 2
aim.target_timeout_ms: 50.0
```

两个现有启动配置均已更新：`src/rm_vision_bringup/config/node_params.yaml` 和 `src/rm_serial_driver/config/serial_driver.yaml`。若需要更窄的世界观测区间，可明确配置world_pitch_min/max；越界后发撤销，绝不把原目标截到另一个角度。

常见状态：`ok_world_pitch`、`no_tracking_target`、`stale_target`、`invalid_or_future_target_stamp`、`invalid_armor_count`、`world_pitch_out_of_domain`、`plan_derivative_limit`、`a6_age_budget_exhausted`，或具体弹道/规划异常文本。

## 本机验证

```bash
python3 tools/test_world_contract.py
# 有配套下位机源码时，额外编译真实MCU解析器，完全在PC上运行：
python3 tools/test_world_contract.py --firmware /path/to/infantry2.1
```

GCC13.2/C++14：原串口/MPC回归通过，世界契约76条检查通过；上位机真实打包的15条A6向量进入真实MCU组帧/解析代码，68条检查通过。覆盖日志10个世界Pitch值、机械不可达、拆包、CRC错误、撤销、TTL/年龄、NaN/Inf、浮点端点及序号端点。只是主机测试桩，没有连接USB或使能电机。

本机未做Ubuntu ROS 2完整构建、节点运行或硬件验证。改动涉及ROS构造参数与发布者，必须在视觉电脑完成真实构建，不能把上述主机测试等同于节点已运行。

## 视觉电脑落地

现场日志的越界拒绝路径，与上游35ca5d5硬截断实现不同。交接工具默认只检查，要求受影响旧文件逐个匹配上游哈希；不匹配就停止，不覆盖现场改动。先记录现场 `git log -1 --oneline` 与 `git status --short`，确认到底使用哪份源文件和install目录。

在**明确选定的源码副本**通过补丁检查后，再应用及编译：

```bash
python3 apply_patch.py --workspace /home/rm/rm_vision_mpc/rm_vision
python3 apply_patch.py --workspace /home/rm/rm_vision_mpc/rm_vision --apply
cd /home/rm/rm_vision_mpc/rm_vision
source tools/setup_local.sh
colcon build --packages-select rm_serial_driver rm_vision_bringup --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rm_serial_driver --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
```

补丁工具不启动ROS、不打开串口、不烧录。若哈希不一致，先对比现场改动再移植，不使用强制覆盖。构建后还需确认实际加载的新参数、planner_status以及MCU的VOFA执行状态；C模式和运动测试另行确认条件后开展。
