#!/usr/bin/env python3
"""Evaluate OpenVINO FP32 or INT8 OBB accuracy and CPU latency."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import openvino as ov
import yaml

from validate_opencv_dnn import (
    DEFAULT_DATASET,
    DEFAULT_MODEL,
    average_precision,
    decode,
    letterbox,
    load_classes,
    load_gt,
    percentile,
    polygon_iou,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL.with_name("yolo26_obb_fp32.xml"),
    )
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--conf-threshold", type=float, default=0.25)
    parser.add_argument("--nms-threshold", type=float, default=0.5)
    parser.add_argument("--match-iou", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    dataset = args.dataset
    classes = load_classes(dataset)
    config = yaml.safe_load((dataset / "dataset.yaml").read_text(encoding="utf-8"))
    if len(config["names"]) != len(classes):
        raise RuntimeError("Class count mismatch")

    core = ov.Core()
    model = core.read_model(str(args.model))
    input_name = model.input(0).get_any_name()
    compiled = core.compile_model(model, "CPU")

    image_paths = sorted((dataset / "images/val").glob("*.jpg"))
    if args.limit:
        image_paths = image_paths[: args.limit]

    gt_counts = {class_id: 0 for class_id in range(len(classes))}
    ap_records: dict[int, list[tuple[float, bool]]] = defaultdict(list)
    true_positives = false_positives = false_negatives = 0
    preprocess_times: list[float] = []
    infer_times: list[float] = []
    postprocess_times: list[float] = []

    for frame_index, image_path in enumerate(image_paths, start=1):
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]

        started = time.perf_counter()
        canvas, scale, pad_x, pad_y = letterbox(image)
        blob = cv2.dnn.blobFromImage(
            canvas, 1.0 / 255.0, (640, 640), swapRB=True
        ).astype(np.float32)
        preprocess_ms = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        output = np.asarray(compiled({input_name: blob})[0])
        infer_ms = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
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
            dataset / "labels/val" / f"{image_path.stem}.txt",
            (width, height),
        )
        for class_id, _ in gt:
            gt_counts[class_id] += 1

        local_counts: dict[int, int] = defaultdict(int)
        entries = []
        for class_id, polygon in gt:
            entries.append((class_id, polygon, local_counts[class_id]))
            local_counts[class_id] += 1
        matched = {
            class_id: [False] * count for class_id, count in local_counts.items()
        }
        for detection in sorted(detections, key=lambda item: item.score, reverse=True):
            best = None
            best_iou = 0.0
            for class_id, polygon, local_index in entries:
                if class_id != detection.class_id or matched[class_id][local_index]:
                    continue
                iou = polygon_iou(detection.polygon, polygon)
                if iou > best_iou:
                    best_iou = iou
                    best = (class_id, local_index)
            if best is not None and best_iou >= args.match_iou:
                class_id, local_index = best
                matched[class_id][local_index] = True
                ap_records[detection.class_id].append((detection.score, True))
                true_positives += 1
            else:
                ap_records[detection.class_id].append((detection.score, False))
                false_positives += 1

        false_negatives += sum(
            count - sum(matched.get(class_id, []))
            for class_id, count in local_counts.items()
        )
        postprocess_times.append((time.perf_counter() - started) * 1000)
        preprocess_times.append(preprocess_ms)
        infer_times.append(infer_ms)
        if frame_index % 50 == 0 or frame_index == len(image_paths):
            print(f"processed {frame_index}/{len(image_paths)}")

    class_ap = {
        classes[class_id]: average_precision(records, gt_counts[class_id])
        for class_id, records in ap_records.items()
    }
    evaluated_classes = [index for index, count in gt_counts.items() if count > 0]
    map50 = statistics.mean(
        class_ap.get(classes[index], 0.0) for index in evaluated_classes
    )
    precision = true_positives / max(1, true_positives + false_positives)
    recall = true_positives / max(1, true_positives + false_negatives)
    f1 = 2 * precision * recall / max(1e-9, precision + recall)
    metrics = {
        "model": str(args.model),
        "model_size_bytes": args.model.stat().st_size,
        "image_count": len(image_paths),
        "map50": map50,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "class_ap50": class_ap,
        "timing_ms": {
            "preprocess_mean": statistics.mean(preprocess_times),
            "inference_mean": statistics.mean(infer_times),
            "inference_p50": statistics.median(infer_times),
            "inference_p95": percentile(infer_times, 0.95),
            "postprocess_mean": statistics.mean(postprocess_times),
            "estimated_fps": 1000.0 / statistics.mean(infer_times),
        },
    }
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
