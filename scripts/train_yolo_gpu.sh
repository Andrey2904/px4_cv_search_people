#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/dron/px4_offboard_clean_ws"
VENV_PYTHON="${ROOT_DIR}/.venv-yolo/bin/python"
TRAIN_SCRIPT="${ROOT_DIR}/tools/train_yolo.py"

DATA="${DATA:-${ROOT_DIR}/training/yolo/data_person.yaml}"
MODEL="${MODEL:-yolo12n.pt}"
IMGSZ="${IMGSZ:-960}"
EPOCHS="${EPOCHS:-60}"
BATCH="${BATCH:-16}"
PATIENCE="${PATIENCE:-15}"
WORKERS="${WORKERS:-8}"
DEVICE="${DEVICE:-0}"
PROJECT="${PROJECT:-${ROOT_DIR}/models/yolo_runs}"
NAME="${NAME:-yolo12n_person_detector_gpu}"
CACHE="${CACHE:-disk}"
OPTIMIZER="${OPTIMIZER:-auto}"
SEED="${SEED:-42}"
COS_LR="${COS_LR:-0}"
RESUME="${RESUME:-0}"

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "Python venv not found: ${VENV_PYTHON}" >&2
    exit 1
fi

if [[ ! -f "${DATA}" ]]; then
    echo "Dataset YAML not found: ${DATA}" >&2
    exit 1
fi

ARGS=(
    "${TRAIN_SCRIPT}"
    --data "${DATA}"
    --model "${MODEL}"
    --imgsz "${IMGSZ}"
    --epochs "${EPOCHS}"
    --batch "${BATCH}"
    --patience "${PATIENCE}"
    --workers "${WORKERS}"
    --device "${DEVICE}"
    --project "${PROJECT}"
    --name "${NAME}"
    --cache "${CACHE}"
    --optimizer "${OPTIMIZER}"
    --seed "${SEED}"
)

if [[ "${COS_LR}" == "1" ]]; then
    ARGS+=(--cos-lr)
fi

if [[ "${RESUME}" == "1" ]]; then
    ARGS+=(--resume)
fi

echo "Starting YOLO training with:"
echo "  DATA=${DATA}"
echo "  MODEL=${MODEL}"
echo "  IMGSZ=${IMGSZ}"
echo "  EPOCHS=${EPOCHS}"
echo "  BATCH=${BATCH}"
echo "  DEVICE=${DEVICE}"
echo "  PROJECT=${PROJECT}"
echo "  NAME=${NAME}"

exec "${VENV_PYTHON}" "${ARGS[@]}"
