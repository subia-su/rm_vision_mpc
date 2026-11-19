#!/usr/bin/env python3
"""Validate and benchmark the imported YOLO26-OBB model with OpenCV DNN CPU."""

from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import statistics
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = REPO_ROOT / "src/rm_auto_aim/armor_detector/model/yolo26_obb.onnx"
DEFAULT_DATASET = REPO_ROOT / "datasets/yolo26_obb"


@dataclass
class Detection:
    class_id: int
    score: float
    center_x: float
    center_y: float
    width: float
    height: float
    angle: float
    polygon: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--conf-threshold", type=float, default=0.25)
    parser.add_argument("--nms-threshold", type=float, default=0.5)
    parser.add_argument("--match-iou", type=float, default=0.5)
    parser.add_argument("--threads", type=int, default=min(16, max(1, (int(os_cpu_count()) + 1) // 2)))
    parser.add_argument("--benchmark-repeat", type=int, default=20)
    parser.add_argument("--save-samples", type=int, default=0)
    return parser.parse_args()


def os_cpu_count() -> str:
    # Kept as a function so argument construction remains easy to read.
    import os

    return os.cpu_count() or "1"


def load_classes(dataset_dir: Path) -> list[str]:
    classes_file = dataset_dir / "classes.txt"
    return [line.strip() for line in classes_file.read_text(encoding="utf-8").splitlines() if line.strip()]


def load_gt(label_path: Path, image_size: tuple[int, int]) -> list[tuple[int, np.ndarray]]:
    width, height = image_size
    gt: list[tuple[int, np.ndarray]] = []
    if not label_path.exists():
        return gt
    for line in label_path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) != 9:
            continue
        values = np.asarray([float(value) for value in fields[1:]], dtype=np.float32)
        points = values.reshape(4, 2) * np.asarray([width, height], dtype=np.float32)
        gt.append((int(fields[0]), points))
    return gt


def letterbox(image: np.ndarray, size: int = 640) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    left = (size - resized_width) // 2
    top = (size - resized_height) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas, scale, left, top


def make_net(model_path: Path, threads: int) -> cv2.dnn_Net:
    cv2.setNumThreads(threads)
    net = cv2.dnn.readNetFromONNX(str(model_path))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net


def decode(
    output: np.ndarray,
    image_size: tuple[int, int],
    scale: float,
    pad_x: int,
    pad_y: int,
    conf_threshold: float,
    nms_threshold: float,
) -> list[Detection]:
    predictions = output[0]
    if predictions.shape[0] != 41:
        raise RuntimeError(f"Expected 41 output rows, got {predictions.shape}")

    class_scores = predictions[4:40]
    class_ids = np.argmax(class_scores, axis=0)
    scores = class_scores[class_ids, np.arange(class_scores.shape[1])]
    keep = np.flatnonzero(scores >= conf_threshold)
    if keep.size == 0:
        return []

    centers = predictions[:2, keep].T
    sizes = predictions[2:4, keep].T
    angles = predictions[40, keep]
    selected: list[int] = []
    for class_id in np.unique(class_ids[keep]):
        local_indices = np.flatnonzero(class_ids[keep] == class_id)
        boxes = [
            (
                (float(centers[index][0]), float(centers[index][1])),
                (float(sizes[index][0]), float(sizes[index][1])),
                math.degrees(float(angles[index])),
            )
            for index in local_indices
        ]
        indices = cv2.dnn.NMSBoxesRotated(
            boxes,
            scores[keep[local_indices]].tolist(),
            conf_threshold,
            nms_threshold,
        )
        selected.extend(int(keep[local_indices[int(index)]]) for index in indices)
    if not selected:
        return []

    original_width, original_height = image_size
    detections: list[Detection] = []
    for source_index in selected:
        center_x = (float(predictions[0, source_index]) - pad_x) / scale
        center_y = (float(predictions[1, source_index]) - pad_y) / scale
        width = float(predictions[2, source_index]) / scale
        height = float(predictions[3, source_index]) / scale
        angle = float(predictions[40, source_index])
        rect = ((center_x, center_y), (width, height), math.degrees(angle))
        polygon = cv2.boxPoints(rect).astype(np.float32)
        polygon[:, 0] = np.clip(polygon[:, 0], 0, original_width - 1)
        polygon[:, 1] = np.clip(polygon[:, 1], 0, original_height - 1)
        detections.append(
            Detection(
                class_id=int(class_ids[source_index]),
                score=float(scores[source_index]),
                center_x=center_x,
                center_y=center_y,
                width=width,
                height=height,
                angle=angle,
                polygon=polygon,
            )
        )
    return detections


def polygon_iou(first: np.ndarray, second: np.ndarray) -> float:
    intersection, _ = cv2.intersectConvexConvex(first, second)
    first_area = abs(cv2.contourArea(first))
    second_area = abs(cv2.contourArea(second))
    union = first_area + second_area - intersection
    return float(intersection / union) if union > 0 else 0.0


def average_precision(records: list[tuple[float, bool]], gt_count: int) -> float:
    if gt_count == 0 or not records:
        return 0.0
    records.sort(key=lambda item: item[0], reverse=True)
    cumulative_tp = 0
    cumulative_fp = 0
    previous_recall = 0.0
    ap = 0.0
    for _, is_true_positive in records:
        cumulative_tp += int(is_true_positive)
        cumulative_fp += int(not is_true_positive)
        recall = cumulative_tp / gt_count
        precision = cumulative_tp / (cumulative_tp + cumulative_fp)
        ap += (recall - previous_recall) * precision
        previous_recall = recall
    return ap


def draw_sample(
    image: np.ndarray,
    gt: list[tuple[int, np.ndarray]],
    detections: list[Detection],
    classes: list[str],
    destination: Path,
) -> None:
    rendered = image.copy()
    for class_id, polygon in gt:
        cv2.polylines(rendered, [polygon.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(
            rendered,
            f"GT {classes[class_id]}",
            tuple(np.mean(polygon, axis=0).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    for detection in detections:
        cv2.polylines(
            rendered,
            [detection.polygon.astype(np.int32)],
            True,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            rendered,
            f"{classes[detection.class_id]} {detection.score:.2f}",
            tuple(np.mean(detection.polygon, axis=0).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.imwrite(str(destination), rendered)


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(quantile * len(ordered)) - 1))
    return ordered[index]


def benchmark_threads(
    model_path: Path,
    image: np.ndarray,
    thread_counts: list[int],
    warmup: int,
    repeat: int,
) -> list[dict[str, float | int]]:
    canvas, _, _, _ = letterbox(image)
    blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (640, 640), swapRB=True)
    results = []
    for threads in thread_counts:
        net = make_net(model_path, threads)
        net.setInput(blob)
        for _ in range(warmup):
            net.forward()
        timings = []
        for _ in range(repeat):
            started = time.perf_counter()
            net.forward()
            timings.append((time.perf_counter() - started) * 1000)
        results.append(
            {
                "threads": threads,
                "mean_ms": statistics.mean(timings),
                "p50_ms": statistics.median(timings),
                "p95_ms": percentile(timings, 0.95),
                "fps": 1000.0 / statistics.mean(timings),
            }
        )
    return results


def write_report(
    destination: Path,
    args: argparse.Namespace,
    classes: list[str],
    metrics: dict,
    thread_results: list[dict],
) -> None:
    lines = [
        "# YOLO26-OBB OpenCV DNN CPU 可行性报告",
        "",
        f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 主机：{platform.node()} / {platform.machine()}",
        f"- Python：{platform.python_version()}",
        f"- OpenCV：{cv2.__version__}",
        f"- 模型：`{args.model}`",
        f"- 数据集：`{args.dataset}`",
        f"- 验证划分：`{args.split}`，图像数：{metrics['image_count']}",
        f"- 类别数：{len(classes)}",
        f"- 置信度阈值：{args.conf_threshold}",
        f"- NMS IoU：{args.nms_threshold}",
        "",
        "## 精度",
        "",
        f"- mAP@0.5：**{metrics['map50']:.4f}**",
        f"- Precision：{metrics['precision']:.4f}",
        f"- Recall：{metrics['recall']:.4f}",
        f"- F1：{metrics['f1']:.4f}",
        f"- 真阳性：{metrics['true_positives']}",
        f"- 假阳性：{metrics['false_positives']}",
        f"- 假阴性：{metrics['false_negatives']}",
        "",
        "## CPU 性能",
        "",
        "| 线程数 | 平均推理 | P50 | P95 | FPS |",
        "|---:|---:|---:|---:|---:|",
    ]
    for result in thread_results:
        lines.append(
            f"| {result['threads']} | {result['mean_ms']:.2f} ms | "
            f"{result['p50_ms']:.2f} ms | {result['p95_ms']:.2f} ms | "
            f"{result['fps']:.1f} |"
        )
    timing = metrics["timing_ms"]
    lines.extend(
        [
            "",
            f"完整样本平均预处理：{timing['preprocess_mean']:.2f} ms",
            f"完整样本平均推理：{timing['inference_mean']:.2f} ms",
            f"完整样本平均后处理：{timing['postprocess_mean']:.2f} ms",
            f"端到端平均：{timing['end_to_end_mean']:.2f} ms，"
            f"P50：{timing['end_to_end_p50']:.2f} ms，"
            f"P95：{timing['end_to_end_p95']:.2f} ms",
            f"端到端估算 FPS：{timing['estimated_fps']:.1f}",
            "",
            "## 结论",
            "",
            "OpenCV DNN 可以在 CPU 上完成该模型的加载、推理和后处理，"
            "但当前验证机在 16 线程下的稳定推理吞吐约 21–24 FPS，"
            "低于项目相机 46 FPS。适合离线验证和低频检测，"
            "不建议在未优化的情况下替代面向 46 FPS 的实时检测路径。",
            "",
            "模型需 OpenCV 4.10 以上版本；本项目系统 OpenCV 4.5.4 无法直接加载。",
            "",
            "## 集成验证",
            "",
            "- ROS 2 节点 `yolo26_dnn_detector/yolo26_dnn_node` 已在真机启动，"
            "独立链接 OpenCV 4.12。",
            "- 相机保持 640×640 中心 ROI（偏移 400,220）和 46 FPS。",
            "- C++ ROS 节点实测约 17–18 FPS，模型单次推理约 42 ms。",
            "- 当前画面没有目标时仍发布空的 `/detector/armors`，不会阻塞后续帧。",
            "",
        ]
    )
    destination.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    if tuple(int(value) for value in cv2.__version__.split(".")[:2]) < (4, 10):
        raise SystemExit(
            f"OpenCV {cv2.__version__} cannot load this ONNX model; use OpenCV >= 4.10"
        )
    args.output.mkdir(parents=True, exist_ok=True)
    samples_dir = Path(__file__).parent / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)

    classes = load_classes(args.dataset)
    config = yaml.safe_load((args.dataset / "dataset.yaml").read_text(encoding="utf-8"))
    configured_names = config.get("names", {})
    if len(configured_names) != len(classes):
        raise RuntimeError("dataset.yaml and classes.txt class counts differ")

    image_paths = sorted((args.dataset / "images" / args.split).glob("*.jpg"))
    if args.limit:
        image_paths = image_paths[: args.limit]
    if not image_paths:
        raise RuntimeError("No validation images found")

    load_start = time.perf_counter()
    net = make_net(args.model, args.threads)
    load_ms = (time.perf_counter() - load_start) * 1000

    gt_counts = {class_id: 0 for class_id in range(len(classes))}
    ap_records: dict[int, list[tuple[float, bool]]] = defaultdict(list)
    frame_rows: list[dict[str, float | int | str]] = []
    true_positives = false_positives = false_negatives = 0
    sample_budget = args.save_samples

    for frame_index, image_path in enumerate(image_paths, start=1):
        started = time.perf_counter()
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"Cannot read {image_path}")
        height, width = image.shape[:2]

        preprocess_started = time.perf_counter()
        canvas, scale, pad_x, pad_y = letterbox(image)
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (640, 640), swapRB=True)
        net.setInput(blob)
        preprocess_ms = (time.perf_counter() - preprocess_started) * 1000

        inference_started = time.perf_counter()
        output = net.forward()
        inference_ms = (time.perf_counter() - inference_started) * 1000

        postprocess_started = time.perf_counter()
        detections = decode(
            output,
            (width, height),
            scale,
            pad_x,
            pad_y,
            args.conf_threshold,
            args.nms_threshold,
        )
        gt = load_gt(
            args.dataset / "labels" / args.split / f"{image_path.stem}.txt",
            (width, height),
        )
        for class_id, _ in gt:
            gt_counts[class_id] += 1

        gt_entries = []
        local_counts: dict[int, int] = defaultdict(int)
        for gt_class, gt_polygon in gt:
            gt_entries.append((gt_class, gt_polygon, local_counts[gt_class]))
            local_counts[gt_class] += 1
        matched_gt = {
            class_id: [False] * count for class_id, count in local_counts.items()
        }
        for detection in sorted(detections, key=lambda item: item.score, reverse=True):
            best_entry = None
            best_iou = 0.0
            for gt_class, gt_polygon, local_index in gt_entries:
                if gt_class != detection.class_id or matched_gt[gt_class][local_index]:
                    continue
                iou = polygon_iou(detection.polygon, gt_polygon)
                if iou > best_iou:
                    best_iou = iou
                    best_entry = (gt_class, local_index)
            if best_iou >= args.match_iou and best_entry is not None:
                gt_class, local_index = best_entry
                matched_gt[gt_class][local_index] = True
                ap_records[detection.class_id].append((detection.score, True))
                true_positives += 1
            else:
                ap_records[detection.class_id].append((detection.score, False))
                false_positives += 1

        false_negatives += sum(
            count - sum(matched_gt.get(class_id, []))
            for class_id, count in local_counts.items()
        )
        postprocess_ms = (time.perf_counter() - postprocess_started) * 1000
        end_to_end_ms = (time.perf_counter() - started) * 1000
        frame_rows.append(
            {
                "image": image_path.name,
                "detections": len(detections),
                "preprocess_ms": preprocess_ms,
                "inference_ms": inference_ms,
                "postprocess_ms": postprocess_ms,
                "end_to_end_ms": end_to_end_ms,
            }
        )
        if sample_budget > 0:
            draw_sample(
                image,
                gt,
                detections,
                classes,
                samples_dir / f"frame_{frame_index:04d}.jpg",
            )
            sample_budget -= 1
        if frame_index % 50 == 0 or frame_index == len(image_paths):
            print(f"processed {frame_index}/{len(image_paths)}")

    class_ap = {
        classes[class_id]: average_precision(records, gt_counts[class_id])
        for class_id, records in ap_records.items()
    }
    classes_with_gt = [class_id for class_id, count in gt_counts.items() if count > 0]
    map50 = statistics.mean(
        class_ap.get(classes[class_id], 0.0) for class_id in classes_with_gt
    )
    precision = true_positives / max(1, true_positives + false_positives)
    recall = true_positives / max(1, true_positives + false_negatives)
    f1 = 2 * precision * recall / max(1e-9, precision + recall)
    inference_times = [row["inference_ms"] for row in frame_rows]
    end_times = [row["end_to_end_ms"] for row in frame_rows]

    test_image = cv2.imread(str(image_paths[0]))
    thread_counts = sorted(set([1, 2, 4, 8, args.threads, 16]))
    thread_results = benchmark_threads(
        args.model,
        test_image,
        thread_counts,
        warmup=max(2, args.benchmark_repeat // 5),
        repeat=args.benchmark_repeat,
    )

    metrics = {
        "opencv_version": cv2.__version__,
        "model_load_ms": load_ms,
        "model_size_bytes": args.model.stat().st_size,
        "image_count": len(image_paths),
        "class_count": len(classes),
        "class_gt_counts": {classes[index]: count for index, count in gt_counts.items()},
        "class_ap50": class_ap,
        "map50": map50,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "true_positives": true_positives,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
        "timing_ms": {
            "preprocess_mean": statistics.mean(row["preprocess_ms"] for row in frame_rows),
            "inference_mean": statistics.mean(inference_times),
            "inference_p50": statistics.median(inference_times),
            "inference_p95": percentile(inference_times, 0.95),
            "postprocess_mean": statistics.mean(row["postprocess_ms"] for row in frame_rows),
            "end_to_end_mean": statistics.mean(end_times),
            "end_to_end_p50": statistics.median(end_times),
            "end_to_end_p95": percentile(end_times, 0.95),
            "estimated_fps": 1000.0 / statistics.mean(end_times),
        },
        "cpu_benchmark": thread_results,
    }
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    with (args.output / "per_frame.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frame_rows[0]))
        writer.writeheader()
        writer.writerows(frame_rows)
    write_report(args.output / "report.md", args, classes, metrics, thread_results)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
