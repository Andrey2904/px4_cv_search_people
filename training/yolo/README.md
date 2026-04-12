## YOLO Fine-Tuning

This directory contains the minimal training assets for fine-tuning a
single-class YOLO detector on the local drone dataset.

Expected workflow:

1. Create a Python environment and install the training dependencies.
2. Audit the dataset with `tools/audit_yolo_dataset.py`.
3. Run training with `scripts/train_yolo_gpu.sh`.
4. Export the best checkpoint to ONNX with `tools/export_yolo_onnx.py`.

Default dataset:

- `/home/dron/.gz/models/datasets/dino_autolabel_combined_annotation_pool_20260407_tuned_split`

Default output root:

- `/home/dron/px4_offboard_clean_ws/models/yolo_runs`

One-command training:

```bash
cd /home/dron/px4_offboard_clean_ws
bash scripts/train_yolo_gpu.sh
```

Override defaults through environment variables:

```bash
cd /home/dron/px4_offboard_clean_ws
NAME=person_detector_v2 EPOCHS=100 BATCH=12 DEVICE=0 bash scripts/train_yolo_gpu.sh
```

The ROS detector in this repository expects an ONNX model path and can run with
`model_architecture:=yolo`.
