# YOLO26 OBB OpenCV DNN CPU feasibility

This directory validates the imported YOLO26-OBB ONNX model without
TensorRT or CUDA.

## Dataset

The complete dataset is imported at:

```text
datasets/yolo26_obb/
├── dataset.yaml
├── classes.txt
├── images/{train,val}/
└── labels/{train,val}/
```

Image and label directories are intentionally ignored by Git because they
total about 795 MB. The dataset configuration and class list are tracked.

## Environment

The model uses ONNX opset 13 and contains `Split`, `Cos`, and `Sin` nodes.
OpenCV 4.5.4 cannot parse the model, so this experiment requires OpenCV
4.10 or newer. It was validated with OpenCV 4.12.0.

```bash
cd /home/rm/rm_vision/rm_vision/experiments/yolo26_onnx_cpu
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run validation

```bash
.venv/bin/python validate_opencv_dnn.py
```

Useful options:

```bash
.venv/bin/python validate_opencv_dnn.py --limit 100
.venv/bin/python validate_opencv_dnn.py --conf-threshold 0.15 --nms-threshold 0.5
.venv/bin/python validate_opencv_dnn.py --save-samples 5
```

Generated files:

- `results/metrics.json`: model, dataset, mAP, precision/recall and timing
- `results/per_frame.csv`: preprocessing, inference and postprocessing timing
- `results/report.md`: human-readable feasibility and performance report
- `samples/`: optional annotated validation images

## Model contract

- Input: RGB float32 `[1, 3, 640, 640]`, scaled to `[0, 1]`
- Output: `[1, 41, 8400]`
- Feature rows: `cx`, `cy`, `width`, `height`, 36 class scores, angle
- Coordinates and angle are in the 640x640 letterboxed image

## Standalone C++ benchmark

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --parallel
cpp/build/yolo26_cpu_benchmark \
  ../../datasets/yolo26_obb/images/val/382.jpg \
  ../../src/rm_auto_aim/armor_detector/model/yolo26_obb.onnx \
  ../../src/rm_auto_aim/armor_detector/model/yolo26_classes.txt \
  20 16 samples/cpp_result.jpg blue
```

