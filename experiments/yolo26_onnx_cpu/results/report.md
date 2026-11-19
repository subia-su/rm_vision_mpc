# YOLO26-OBB OpenCV DNN CPU 可行性报告

- 生成时间：2026-11-19 16:55:44
- 主机：rm-D80 / x86_64
- Python：3.10.12
- OpenCV：4.12.0
- 模型：`/home/rm/rm_vision/rm_vision/src/rm_auto_aim/armor_detector/model/yolo26_obb.onnx`
- 数据集：`/home/rm/rm_vision/rm_vision/datasets/yolo26_obb`
- 验证划分：`val`，图像数：607
- 类别数：36
- 置信度阈值：0.25
- NMS IoU：0.5

## 精度

- mAP@0.5：**0.5995**
- Precision：0.8257
- Recall：0.8962
- F1：0.8595
- 真阳性：976
- 假阳性：206
- 假阴性：113

## CPU 性能

| 线程数 | 平均推理 | P50 | P95 | FPS |
|---:|---:|---:|---:|---:|
| 1 | 133.98 ms | 133.80 ms | 136.50 ms | 7.5 |
| 2 | 73.20 ms | 73.26 ms | 73.66 ms | 13.7 |
| 4 | 48.34 ms | 47.92 ms | 51.06 ms | 20.7 |
| 8 | 48.73 ms | 48.72 ms | 49.59 ms | 20.5 |
| 16 | 41.63 ms | 41.78 ms | 43.87 ms | 24.0 |

完整样本平均预处理：1.73 ms
完整样本平均推理：48.74 ms
完整样本平均后处理：0.89 ms
端到端平均：53.58 ms，P50：53.38 ms，P95：57.22 ms
端到端估算 FPS：18.7

## 结论

OpenCV DNN 可以在 CPU 上完成该模型的加载、推理和后处理，但当前验证机在 16 线程下的稳定推理吞吐约 21–24 FPS，低于项目相机 46 FPS。适合离线验证和低频检测，不建议在未优化的情况下替代面向 46 FPS 的实时检测路径。

模型需 OpenCV 4.10 以上版本；本项目系统 OpenCV 4.5.4 无法直接加载。

## 集成验证

- 已新增 ROS 2 节点 `yolo26_dnn_detector/yolo26_dnn_node`，独立链接 OpenCV 4.12。
- 真机启动成功，相机 ROI 为 640×640、偏移 `(400,220)`、46 Hz。
- C++ ROS 节点实际输出约 17–18 FPS，模型单次推理约 42 ms。
- C++ 独立样本验证成功：`blue_infantry_5` 置信度约 0.954。
- 当前画面没有目标时仍正常发布空的 `/detector/armors`，不会阻塞后续帧。
