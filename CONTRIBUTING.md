# 队员协作

主仓库：[xkddyl/rm_vision_mpc](https://github.com/xkddyl/rm_vision_mpc)。上游开源保留为参考，不直接向上游提交本队改动。

```bash
git clone https://github.com/xkddyl/rm_vision_mpc.git
cd rm_vision_mpc
git switch -c fix/你的任务名
```

检测、跟踪、串口接口分别按 `src/rm_auto_aim`、`src/rm_serial_driver` 分工；常用参数在 `src/rm_vision_bringup/config/node_params.yaml`。同步之前先提交或暂存自己的修改，不用整包覆盖别人的文件。

接口修改先运行 `python3 tools/test_world_contract.py`。依赖、ROS构建及世界Pitch契约见 [README](README.md) 和 [接口说明](docs/world_pitch_contract_20261127.md)。完整ROS构建需要Ubuntu/ROS环境，主机测试不替代实车验证。

```bash
git add 你修改的文件
git commit -m "说明改动的具体行为"
git push -u origin HEAD
```

有本仓库写权限的队员推送自己的分支，再用Pull Request汇总；没有写权限的队员可从本仓库fork后提交PR。发给队友时优先发送本仓库的分支或提交链接，便于确认版本。

当前代码包含世界Pitch修复及PC验证；Ubuntu完整构建、安装外参、Yaw原点及新自动分支的实车运动尚未验收。不要把“main能下载”当作这些验收项已经完成。
