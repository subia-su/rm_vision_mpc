#!/usr/bin/env python3
"""INT8 quantization with NNCF accuracy control using the OBB mAP metric."""

from __future__ import annotations

import argparse
import statistics
import time
from collections import defaultdict
from pathlib import Path

import cv2
import nncf
import numpy as np
import openvino as ov

from quantize_openvino import preprocess, select_balanced_images
from validate_opencv_dnn import (
    DEFAULT_DATASET,
    DEFAULT_MODEL,
    average_precision,
    decode,
    letterbox,
    load_gt,
    polygon_iou,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fp32-xml",
        type=Path,
        default=DEFAULT_MODEL.with_name("yolo26_obb_fp32.xml"),
    )
    parser.add_argument(
        "--output-xml",
        type=Path,
        default=DEFAULT_MODEL.with_name("yolo26_obb_int8_ac.xml"),
    )
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--calibration-size", type=int, default=300)
    parser.add_argument("--validation-size", type=int, default=200)
    parser.add_argument("--max-drop", type=float, default=0.01)
    parser.add_argument("--seed", type=int, default=2026)
    return parser.parse_args()


def select_validation_images(dataset: Path, count: int, seed: int) -> list[Path]:
    paths = sorted((dataset / "images/val").glob("*.jpg"))
    if len(paths) <= count:
        return paths
    step = len(paths) / count
    selected = [paths[min(len(paths) - 1, int(index * step))] for index in range(count)]
    # Deterministic and evenly covers the entire validation set.
    return sorted(set(selected), key=paths.index)[:count]


def evaluate_map50(
    compiled_model: ov.CompiledModel,
    input_name: str,
    dataset: Path,
    image_paths: list[Path],
) -> float:
    gt_counts: dict[int, int] = defaultdict(int)
    ap_records: dict[int, list[tuple[float, bool]]] = defaultdict(list)

    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        canvas, scale, pad_x, pad_y = letterbox(image)
        blob = cv2.dnn.blobFromImage(
            canvas, 1.0 / 255.0, (640, 640), swapRB=True
        ).astype(np.float32)
        output = np.asarray(compiled_model({input_name: blob})[0])
        detections = decode(output, (width, height), scale, pad_x, pad_y, 0.25, 0.5)
        gt = load_gt(dataset / "labels/val" / f"{image_path.stem}.txt", (width, height))

        local_counts: dict[int, int] = defaultdict(int)
        entries = []
        for class_id, polygon in gt:
            gt_counts[class_id] += 1
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
            if best is not None and best_iou >= 0.5:
                class_id, local_index = best
                matched[class_id][local_index] = True
                ap_records[detection.class_id].append((detection.score, True))
            else:
                ap_records[detection.class_id].append((detection.score, False))

    class_ap = [
        average_precision(records, gt_counts[class_id])
        for class_id, records in ap_records.items()
    ]
    for class_id, count in gt_counts.items():
        if count > 0 and not ap_records.get(class_id):
            class_ap.append(0.0)
    return statistics.mean(class_ap) if class_ap else 0.0


def main() -> None:
    args = parse_args()
    core = ov.Core()
    model = core.read_model(str(args.fp32_xml))
    input_name = model.input(0).get_any_name()
    calibration_paths = select_balanced_images(
        args.dataset, args.calibration_size, args.seed
    )
    validation_paths = select_validation_images(
        args.dataset, args.validation_size, args.seed
    )
    calibration_dataset = nncf.Dataset(
        calibration_paths,
        lambda path: preprocess(path, input_name),
    )
    validation_dataset = nncf.Dataset(
        validation_paths,
        lambda path: preprocess(path, input_name),
    )

    validation_count = 0

    def validate(
        compiled_model: ov.CompiledModel,
        _: nncf.Dataset,
    ) -> float:
        nonlocal validation_count
        started = time.perf_counter()
        score = evaluate_map50(
            compiled_model,
            input_name,
            args.dataset,
            validation_paths,
        )
        validation_count += 1
        print(
            f"validation[{validation_count}] "
            f"mAP50={score:.6f} elapsed={time.perf_counter() - started:.1f}s"
        )
        return score

    quantized_model = nncf.quantize_with_accuracy_control(
        model,
        calibration_dataset=calibration_dataset,
        validation_dataset=validation_dataset,
        validation_fn=validate,
        max_drop=args.max_drop,
        drop_type=nncf.DropType.ABSOLUTE,
        subset_size=min(args.calibration_size, len(calibration_paths)),
        preset=nncf.QuantizationPreset.MIXED,
        target_device=nncf.TargetDevice.CPU,
        fast_bias_correction=False,
    )
    ov.save_model(
        quantized_model,
        str(args.output_xml),
        compress_to_fp16=False,
    )
    print(f"Saved accuracy-controlled INT8 IR: {args.output_xml}")


if __name__ == "__main__":
    main()
