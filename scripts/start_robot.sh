#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-damping}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CAN_IFACES="${CAN_IFACES:-can0 can1 can2 can3}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
COMMAND_FILE="${COMMAND_FILE:-/tmp/opendoge_command.state}"
IMU_FILE="${IMU_FILE:-/tmp/opendoge_imu.state}"
CONFIG_FILE="${CONFIG_FILE:-${ROOT_DIR}/src/opendoge_deploy/configs/opendoge_deploy.conf}"
DEPLOY_BIN="${DEPLOY_BIN:-${ROOT_DIR}/install/opendoge_deploy/bin/opendoge_deploy}"
POLICY_BACKEND="${POLICY_BACKEND:-onnx}"
POLICY_PATH="${POLICY_PATH:-}"
IMU_DEVICE="${IMU_DEVICE:-/dev/ttyUSB0}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
REALTIME_ARGS="${REALTIME_ARGS:-}"

PIDS=()

usage() {
  cat <<EOF
Usage: $0 [dry|damping|policy]

Environment:
  CAN_IFACES       CAN interfaces, default: "can0 can1 can2 can3"
  CAN_BITRATE      CAN bitrate, default: 1000000
  COMMAND_FILE     command.state path, default: /tmp/opendoge_command.state
  IMU_FILE         imu.state path, default: /tmp/opendoge_imu.state
  CONFIG_FILE      deploy config path
  POLICY_PATH      ONNX path, required for policy mode
  POLICY_BACKEND   policy backend, default: onnx
  IMU_DEVICE       DM-IMU serial device, default: /dev/ttyUSB0
  JOYSTICK_DEVICE  optional /dev/input/js* device
  REALTIME_ARGS    optional deploy realtime args, for example: "--realtime --cpu 0"
EOF
}

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -e "${path}" ]]; then
    echo "${label} not found: ${path}" >&2
    exit 1
  fi
}

write_safe_command() {
  mkdir -p "$(dirname "${COMMAND_FILE}")" "$(dirname "${IMU_FILE}")"
  cat >"${COMMAND_FILE}" <<EOF
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
EOF
  cat >"${IMU_FILE}" <<EOF
wx=0.0
wy=0.0
wz=0.0
gx=0.0
gy=0.0
gz=-1.0
EOF
}

start_can() {
  local iface
  for iface in ${CAN_IFACES}; do
    sudo "${ROOT_DIR}/scripts/setup_can.sh" "${iface}" "${CAN_BITRATE}"
  done
}

start_imu_bridge() {
  if [[ -e "${IMU_DEVICE}" ]]; then
    "${ROOT_DIR}/tools/imu/dm_imu_bridge.py" \
      --device "${IMU_DEVICE}" \
      --baud 921600 \
      --configure-usb \
      --output "${IMU_FILE}" &
    PIDS+=("$!")
  else
    echo "IMU device not found, using initial ${IMU_FILE}: ${IMU_DEVICE}" >&2
  fi
}

start_joystick_bridge() {
  local args=(--output "${COMMAND_FILE}" --require-rb)
  if [[ -n "${JOYSTICK_DEVICE}" ]]; then
    args+=(--device "${JOYSTICK_DEVICE}")
  fi
  if compgen -G "/dev/input/js*" >/dev/null || [[ -n "${JOYSTICK_DEVICE}" ]]; then
    "${ROOT_DIR}/tools/joystick/xbox_command_bridge.py" "${args[@]}" &
    PIDS+=("$!")
  else
    echo "Joystick not found, keeping inactive command file: ${COMMAND_FILE}" >&2
  fi
}

run_deploy() {
  local args=("$@")
  require_file "${DEPLOY_BIN}" "deploy binary"
  require_file "${CONFIG_FILE}" "deploy config"
  "${DEPLOY_BIN}" \
    --config "${CONFIG_FILE}" \
    --command-file "${COMMAND_FILE}" \
    --imu-file "${IMU_FILE}" \
    ${REALTIME_ARGS} \
    "${args[@]}"
}

case "${MODE}" in
  -h|--help|help)
    usage
    ;;
  dry)
    write_safe_command
    run_deploy --policy-backend none --duration-sec 2
    ;;
  damping)
    write_safe_command
    start_can
    start_imu_bridge
    start_joystick_bridge
    run_deploy --real --enable --clear-fault --policy-backend none
    ;;
  policy)
    if [[ -z "${POLICY_PATH}" ]]; then
      echo "POLICY_PATH is required in policy mode" >&2
      exit 1
    fi
    require_file "${POLICY_PATH}" "policy"
    write_safe_command
    start_can
    start_imu_bridge
    start_joystick_bridge
    run_deploy --real --enable --clear-fault \
      --policy-backend "${POLICY_BACKEND}" \
      --policy-path "${POLICY_PATH}"
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    usage >&2
    exit 1
    ;;
esac
