#!/usr/bin/env python3
"""Run Ultralytics YOLO prediction on a video file and save the result."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        type=Path,
        required=True,
        help="Path to a trained YOLO .pt checkpoint.",
    )
    parser.add_argument(
        "--source",
        type=Path,
        required=True,
        help="Path to the input video.",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument(
        "--project",
        type=Path,
        default=Path("/home/dron/px4_offboard_clean_ws/models/yolo_runs/predict"),
    )
    parser.add_argument("--name", default="video_preview")
    parser.add_argument("--line-width", type=int, default=2)
    parser.add_argument("--show-labels", action="store_true")
    parser.add_argument("--show-conf", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(
            "Ultralytics is not installed. Use the .venv-yolo environment."
        ) from error

    if not args.weights.is_file():
        raise SystemExit(f"Weights not found: {args.weights}")
    if not args.source.is_file():
        raise SystemExit(f"Video not found: {args.source}")

    args.project.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(args.weights))
    results = model.predict(
        source=str(args.source),
        device=args.device,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        save=True,
        project=str(args.project),
        name=args.name,
        exist_ok=True,
        line_width=args.line_width,
        show_labels=args.show_labels,
        show_conf=args.show_conf,
        stream=False,
    )
    print(f"frames_processed={len(results)}")
    print(f"save_dir={args.project / args.name}")


if __name__ == "__main__":
    main()
