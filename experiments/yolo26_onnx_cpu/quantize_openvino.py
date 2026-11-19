#!/usr/bin/env python3
"""Create FP32 OpenVINO IR and perform calibrated INT8 post-training quantization."""

from __future__ import annotations

import argparse
import random
from collections import defaultdict
from pathlib import Path

import cv2
import nncf
import numpy as np
import openvino as ov

from validate_opencv_dnn import DEFAULT_DATASET, DEFAULT_MODEL, letterbox


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--fp32-xml",
        type=Path,
        default=DEFAULT_MODEL.with_name("yolo26_obb_fp32.xml"),
    )
    parser.add_argument(
        "--int8-xml",
        type=Path,
        default=DEFAULT_MODEL.with_name("yolo26_obb_int8.xml"),
    )
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--calibration-size", type=int, default=300)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--preset",
        choices=("performance", "mixed"),
        default="mixed",
    )
    return parser.parse_args()


def select_balanced_images(dataset: Path, count: int, seed: int) -> list[Path]:
    """Round-robin by first label class so rare classes are represented."""
    buckets: dict[int, list[Path]] = defaultdict(list)
    for label_path in sorted((dataset / "labels/train").glob("*.txt")):
        lines = label_path.read_text(encoding="utf-8").splitlines()
        if not lines:
            continue
        class_id = int(lines[0].split()[0])
        image_path = dataset / "images/train" / f"{label_path.stem}.jpg"
        if image_path.exists():
            buckets[class_id].append(image_path)

    generator = random.Random(seed)
    for paths in buckets.values():
        generator.shuffle(paths)

    selected: list[Path] = []
    class_ids = sorted(buckets)
    while len(selected) < count:
        added = False
        for class_id in class_ids:
            if buckets[class_id]:
                selected.append(buckets[class_id].pop())
                added = True
                if len(selected) == count:
                    break
        if not added:
            break
    return selected


def load_model(model_path: Path) -> ov.Model:
    return ov.Core().read_model(str(model_path))


def preprocess(path: Path, input_name: str) -> dict[str, np.ndarray]:
    image = cv2.imread(str(path))
    if image is None:
        raise RuntimeError(f"Cannot read calibration image {path}")
    canvas, _, _, _ = letterbox(image)
    blob = cv2.dnn.blobFromImage(
        canvas, 1.0 / 255.0, (640, 640), swapRB=True
    ).astype(np.float32)
    return {input_name: blob}


def main() -> None:
    args = parse_args()
    args.fp32_xml.parent.mkdir(parents=True, exist_ok=True)

    if not args.fp32_xml.exists():
        model = ov.convert_model(
            str(args.onnx),
            example_input=np.zeros((1, 3, 640, 640), dtype=np.float32),
        )
        ov.save_model(model, str(args.fp32_xml), compress_to_fp16=False)
        print(f"Saved FP32 IR: {args.fp32_xml}")

    model = load_model(args.fp32_xml)
    input_name = model.input(0).get_any_name()
    image_paths = select_balanced_images(
        args.dataset, args.calibration_size, args.seed
    )
    if not image_paths:
        raise RuntimeError("No calibration images found")
    print(f"Calibration images: {len(image_paths)}")

    calibration_dataset = nncf.Dataset(
        image_paths,
        lambda path: preprocess(path, input_name),
    )
    preset = (
        nncf.QuantizationPreset.MIXED
        if args.preset == "mixed"
        else nncf.QuantizationPreset.PERFORMANCE
    )
    quantized_model = nncf.quantize(
        model,
        calibration_dataset,
        subset_size=min(args.calibration_size, len(image_paths)),
        preset=preset,
        target_device=nncf.TargetDevice.CPU,
        fast_bias_correction=False,
    )
    ov.save_model(
        quantized_model,
        str(args.int8_xml),
        compress_to_fp16=False,
    )

    compiled = ov.Core().compile_model(quantized_model, "CPU")
    sample = preprocess(image_paths[0], input_name)[input_name]
    output = compiled(sample)[0]
    print(
        f"Saved INT8 IR: {args.int8_xml} "
        f"output={output.shape} range=({output.min():.3f}, {output.max():.3f})"
    )


if __name__ == "__main__":
    main()
