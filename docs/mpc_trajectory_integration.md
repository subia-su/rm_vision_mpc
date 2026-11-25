# A6 MPC 轨迹接入

## 工作区与构建

标准工作区：

```text
/home/rm/rm_vision_mpc/rm_vision
```

```bash
cd /home/rm/rm_vision_mpc/rm_vision
source tools/setup_local.sh
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

`/home/rm/rm_vision/rm_vision` 是兼容符号链接。不要从
`rm_vision.legacy-20261124` 或历史 `rm_vision(2)` 目录启动。

## 职责划分

上位机负责：

- EKF 目标状态
- 弹道预测
- 100 点 MPC 优化
- 角度、角速度和角加速度参考
- A6 报文封装和 TTL

下位机负责：

- A6 解析和 CRC 校验
- `valid` 与 TTL 检查
- Pitch DM4310 MIT 力矩前馈
- Yaw GM6020 角度—速度串级控制及前馈
- 电机使能、限幅和故障保护

MPC 当前不使用云台实际状态作为优化输入。预测域为 100 点、步长 10 ms，
输出预测域中点轨迹状态。

## A6 报文

```text
长度: 38 字节
字节 0:    0xA6
字节 1:    valid
字节 2-3:  sequence, uint16 little-endian
字节 4-7:  host_ms, uint32 little-endian
字节 8-9:  age_ms, uint16 little-endian
字节 10-11: ttl_ms, uint16 little-endian
字节 12-35: yaw/pitch/yaw_speed/pitch_speed/yaw_acc/pitch_acc,
           6 × binary32 little-endian
字节 36-37: CRC16
```

默认 `ttl_ms=40`。目标丢失或弹道不可解时发送 `valid=false`，下位机应停止
执行旧轨迹。

## 联调检查

1. 确认 `/tracker/target` 的 `tracking=true`。
2. 确认串口反馈约 100 Hz。
3. 确认 A6 写入约 46 Hz、每次 38 字节。
4. 检查下位机是否收到正确的 `sequence`、`valid`、`yaw` 和 `pitch`。
5. 首次实车联调仍应保持 `C_MANUAL_TEST=1u`；确认协议和方向后再切换到自动执行。

如果 A6 `valid=1`、yaw 指令变化，但云台不动作，优先检查下位机是否仍只解析
旧 A5 报文、是否处于手动测试模式，以及电机使能和 A6 执行分支是否开启。
