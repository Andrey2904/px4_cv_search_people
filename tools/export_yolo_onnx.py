#!/usr/bin/env python3
"""Export a trained Ultralytics checkpoint to ONNX."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        type=Path,
        required=True,
        help="Path to the trained .pt checkpoint.",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--simplify", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(
            "Ultralytics is not installed. Install training/yolo/requirements.txt first."
        ) from error

    if not args.weights.is_file():
        raise SystemExit(f"Checkpoint not found: {args.weights}")

    model = YOLO(str(args.weights))
    export_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        device=args.device,
        half=args.half,
        simplify=args.simplify,
    )
    print(export_path)


if __name__ == "__main__":
    main()
