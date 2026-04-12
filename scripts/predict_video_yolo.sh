#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/dron/px4_offboard_clean_ws"
VENV_PYTHON="${ROOT_DIR}/.venv-yolo/bin/python"
PREDICT_SCRIPT="${ROOT_DIR}/tools/predict_video_yolo.py"

WEIGHTS="${WEIGHTS:-/home/dron/.gz/models/yolo12n_people_package/runs/yolo12n_people_v1_safe2/weights/best.pt}"
SOURCE="${SOURCE:-}"
DEVICE="${DEVICE:-0}"
IMGSZ="${IMGSZ:-960}"
CONF="${CONF:-0.25}"
IOU="${IOU:-0.45}"
PROJECT="${PROJECT:-${ROOT_DIR}/models/yolo_runs/predict}"
NAME="${NAME:-video_preview}"
LINE_WIDTH="${LINE_WIDTH:-2}"
SHOW_LABELS="${SHOW_LABELS:-0}"
SHOW_CONF="${SHOW_CONF:-0}"

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "Python venv not found: ${VENV_PYTHON}" >&2
    exit 1
fi

if [[ -z "${SOURCE}" ]]; then
    echo "Set SOURCE=/absolute/path/to/video.webm" >&2
    exit 1
fi

ARGS=(
    "${PREDICT_SCRIPT}"
    --weights "${WEIGHTS}"
    --source "${SOURCE}"
    --device "${DEVICE}"
    --imgsz "${IMGSZ}"
    --conf "${CONF}"
    --iou "${IOU}"
    --project "${PROJECT}"
    --name "${NAME}"
    --line-width "${LINE_WIDTH}"
)

if [[ "${SHOW_LABELS}" == "1" ]]; then
    ARGS+=(--show-labels)
fi

if [[ "${SHOW_CONF}" == "1" ]]; then
    ARGS+=(--show-conf)
fi

echo "Running YOLO video prediction with:"
echo "  WEIGHTS=${WEIGHTS}"
echo "  SOURCE=${SOURCE}"
echo "  DEVICE=${DEVICE}"
echo "  IMGSZ=${IMGSZ}"
echo "  CONF=${CONF}"
echo "  IOU=${IOU}"
echo "  PROJECT=${PROJECT}"
echo "  NAME=${NAME}"

exec "${VENV_PYTHON}" "${ARGS[@]}"
