#!/usr/bin/env python3
"""Audit a YOLO-format dataset and print a compact quality summary."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-root",
        type=Path,
        required=True,
        help="Path to YOLO dataset root with train/test or train/valid splits.",
    )
    return parser.parse_args()


def find_split_dirs(dataset_root: Path) -> list[str]:
    candidates = ["train", "val", "valid", "test"]
    return [name for name in candidates if (dataset_root / name).is_dir()]


def audit_split(dataset_root: Path, split: str) -> dict[str, object]:
    image_dir = dataset_root / split / "images"
    label_dir = dataset_root / split / "labels"

    image_map = {path.stem: path for path in image_dir.iterdir() if path.is_file()}
    label_map = {path.stem: path for path in label_dir.iterdir() if path.is_file()}

    missing_labels = sorted(set(image_map) - set(label_map))
    missing_images = sorted(set(label_map) - set(image_map))

    class_counts: Counter[int] = Counter()
    empty_labels = 0
    invalid_lines = 0
    invalid_images = 0
    widths: list[float] = []
    heights: list[float] = []
    areas: list[float] = []

    for image_path in image_map.values():
        try:
            with Image.open(image_path) as image:
                image.verify()
        except Exception:
            invalid_images += 1

    for label_path in label_dir.glob("*.txt"):
        lines = [line.strip() for line in label_path.read_text().splitlines() if line.strip()]
        if not lines:
            empty_labels += 1
            continue

        for line in lines:
            parts = line.split()
            if len(parts) != 5:
                invalid_lines += 1
                continue

            try:
                class_id = int(float(parts[0]))
                center_x = float(parts[1])
                center_y = float(parts[2])
                width = float(parts[3])
                height = float(parts[4])
            except ValueError:
                invalid_lines += 1
                continue

            if not (
                0.0 <= center_x <= 1.0
                and 0.0 <= center_y <= 1.0
                and 0.0 < width <= 1.0
                and 0.0 < height <= 1.0
            ):
                invalid_lines += 1
                continue

            class_counts[class_id] += 1
            widths.append(width)
            heights.append(height)
            areas.append(width * height)

    def minmax(values: list[float]) -> tuple[float | None, float | None]:
        if not values:
            return None, None
        return min(values), max(values)

    return {
        "images": len(image_map),
        "labels": len(label_map),
        "missing_labels": len(missing_labels),
        "missing_images": len(missing_images),
        "empty_labels": empty_labels,
        "invalid_lines": invalid_lines,
        "invalid_images": invalid_images,
        "class_counts": dict(class_counts),
        "box_count": sum(class_counts.values()),
        "width_minmax": minmax(widths),
        "height_minmax": minmax(heights),
        "area_minmax": minmax(areas),
    }


def main() -> None:
    args = parse_args()
    splits = find_split_dirs(args.dataset_root)
    if not splits:
        raise SystemExit(f"No dataset splits found under {args.dataset_root}")

    print(f"Dataset root: {args.dataset_root}")
    for split in splits:
        print(f"\n[{split}]")
        summary = audit_split(args.dataset_root, split)
        for key, value in summary.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
