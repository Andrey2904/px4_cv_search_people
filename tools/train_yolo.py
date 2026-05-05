#!/usr/bin/env python3
"""Train a YOLO detector with a reproducible CLI wrapper."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    try:
        import torch
        default_device = "0" if torch.cuda.is_available() else "cpu"
    except Exception:
        default_device = "cpu"

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        type=Path,
        default=Path("/home/dron/px4_offboard_clean_ws/training/yolo/data_person.yaml"),
        help="Path to Ultralytics dataset YAML.",
    )
    parser.add_argument(
        "--model",
        default="yolo12s.pt",
        help="Base checkpoint name or path.",
    )
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--patience", type=int, default=15)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default=default_device)
    parser.add_argument(
        "--project",
        type=Path,
        default=Path("/home/dron/px4_offboard_clean_ws/models/yolo_runs"),
    )
    parser.add_argument("--name", default="yolo12s_person_detector_v1")
    parser.add_argument("--cache", default="disk")
    parser.add_argument("--optimizer", default="auto")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cos-lr", action="store_true")
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(
            "Ultralytics is not installed. Install training/yolo/requirements.txt first."
        ) from error

    args.project.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.model)
    results = model.train(
        data=str(args.data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        patience=args.patience,
        workers=args.workers,
        device=args.device,
        project=str(args.project),
        name=args.name,
        cache=args.cache,
        optimizer=args.optimizer,
        seed=args.seed,
        cos_lr=args.cos_lr,
        resume=args.resume,
        exist_ok=True,
        single_cls=True,
        plots=True,
    )
    print(results)


if __name__ == "__main__":
    main()
