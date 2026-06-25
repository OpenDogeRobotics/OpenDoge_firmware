#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-damping}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CAN_IFACES="${CAN_IFACES:-can0 can1 can2 can3}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
COMMAND_FILE="${COMMAND_FILE:-/tmp/opendoge_command.state}"
IMU_FILE="${IMU_FILE:-/tmp/opendoge_imu.state}"
STATUS_FILE="${STATUS_FILE:-/tmp/opendoge_status.json}"
CONFIG_FILE="${CONFIG_FILE:-${ROOT_DIR}/deploy/configs/opendoge_deploy.conf}"
DEPLOY_BIN="${DEPLOY_BIN:-${ROOT_DIR}/install/opendoge_deploy/bin/opendoge_deploy}"
POLICY_BACKEND="${POLICY_BACKEND:-onnx}"
POLICY_PATH="${POLICY_PATH:-}"
REALTIME_ARGS="${REALTIME_ARGS:-}"
CALIB_OUTPUT="${CALIB_OUTPUT:-/tmp/opendoge_calibration.conf}"
CALIB_CHANNEL="${CALIB_CHANNEL:-can0}"

usage() {
  cat <<EOF
Usage: $0 [dry|damping|policy|calibrate|low-gain|verify]

Modes:
  dry         Dry-run deploy (no hardware, no policy, 2s)
  damping     Real deploy with PD damping (no policy)
  policy      Real deploy with ONNX policy inference
  low-gain    Real deploy, low-gain static standing test
  calibrate   Motor calibration on a single CAN channel
  verify      Check all services and interfaces are ready, then exit

Prerequisites:
  CAN interfaces (can0-3): managed by opendoge-can.service
  IMU bridge:              managed by opendoge-imu.service (user)
  Joystick bridge:         managed by opendoge-joystick.service (user)
  Xbox driver:             managed by opendoge-xboxdrv.service

  Install all services once with: bash scripts/install_services.sh

Environment:
  CAN_IFACES       CAN interfaces, default: "can0 can1 can2 can3"
  CAN_BITRATE      CAN bitrate, default: 1000000
  COMMAND_FILE     command.state path, default: /tmp/opendoge_command.state
  IMU_FILE         imu.state path, default: /tmp/opendoge_imu.state
  CONFIG_FILE      deploy config path
  POLICY_PATH      ONNX path, required for policy mode
  POLICY_BACKEND   policy backend, default: onnx
  REALTIME_ARGS    optional deploy realtime args, e.g. "--realtime --cpu 0"
  CALIB_OUTPUT     calibration output path, default: /tmp/opendoge_calibration.conf
  CALIB_CHANNEL    CAN channel for calibration, default: can0
EOF
}

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
  # Only seed files if they don't exist (services may already have written real data)
  if [[ ! -s "${COMMAND_FILE}" ]]; then
    cat >"${COMMAND_FILE}" <<EOF
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
clear_fault=false
low_gain_mode=false
EOF
  fi
  if [[ ! -s "${IMU_FILE}" ]]; then
    cat >"${IMU_FILE}" <<EOF
wx=0.0
wy=0.0
wz=0.0
gx=0.0
gy=0.0
gz=-1.0
EOF
  fi
}

verify_can() {
  local ok=true
  local iface
  for iface in ${CAN_IFACES}; do
    if ! ip link show "${iface}" &>/dev/null; then
      echo "  ✗ ${iface} does not exist (check USB-CAN adapters)" >&2
      ok=false
    elif ip link show "${iface}" | grep -q "state UP"; then
      echo "  ✓ ${iface} UP"
    else
      echo "  ✗ ${iface} exists but is DOWN — restart with: sudo systemctl restart opendoge-can" >&2
      ok=false
    fi
  done
  if ! ${ok}; then
    echo ""
    echo "Troubleshooting:" >&2
    echo "  sudo systemctl status opendoge-can" >&2
    echo "  sudo journalctl -u opendoge-can -n 20" >&2
    echo "  lsusb | grep 1d50:606f     # should show 4 CANable adapters" >&2
    return 1
  fi
  return 0
}

verify_service() {
  local service="$1"
  local manager="$2"  # "system" or "user"
  local label="$3"

  if [[ "${manager}" == "user" ]]; then
    if systemctl --user is-active --quiet "${service}" 2>/dev/null; then
      echo "  ✓ ${label} (${service}) active"
      return 0
    else
      echo "  ✗ ${label} (${service}) not running — restart with: systemctl --user restart ${service}" >&2
      return 1
    fi
  else
    if systemctl is-active --quiet "${service}" 2>/dev/null; then
      echo "  ✓ ${label} (${service}) active"
      return 0
    else
      echo "  ✗ ${label} (${service}) not running — restart with: sudo systemctl restart ${service}" >&2
      return 1
    fi
  fi
}

verify_all() {
  local all_ok=true

  echo "Verifying services and interfaces ..."
  echo ""

  echo "CAN interfaces:"
  verify_can || all_ok=false

  echo ""
  echo "Daemon services:"
  verify_service "opendoge-xboxdrv" "system" "Xbox driver" || all_ok=false
  verify_service "opendoge-joystick" "user" "Joystick bridge" || all_ok=false
  verify_service "opendoge-imu" "user" "IMU bridge" || all_ok=false

  echo ""
  if ${all_ok}; then
    echo "✓ All services ready."
    return 0
  else
    echo "✗ Some services are not ready. See above for restart commands." >&2
    return 1
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
    --status-file "${STATUS_FILE}" \
    ${REALTIME_ARGS} \
    "${args[@]}"
}

case "${MODE}" in
  -h|--help|help)
    usage
    ;;

  verify)
    verify_all
    ;;

  dry)
    write_safe_command
    run_deploy --policy-backend none --duration-sec 2
    ;;

  damping)
    verify_all
    write_safe_command
    run_deploy --real --enable --clear-fault --policy-backend none
    ;;

  policy)
    if [[ -z "${POLICY_PATH}" ]]; then
      echo "POLICY_PATH is required in policy mode" >&2
      exit 1
    fi
    require_file "${POLICY_PATH}" "policy"
    verify_all
    write_safe_command
    run_deploy --real --enable --clear-fault \
      --policy-backend "${POLICY_BACKEND}" \
      --policy-path "${POLICY_PATH}"
    ;;

  calibrate)
    echo "Verifying CAN interfaces ..."
    verify_can
    echo ""
    echo "Running motor calibration on ${CALIB_CHANNEL}..."
    "${ROOT_DIR}/hardware/motor/el05_calibrate.py" \
      --channel "${CALIB_CHANNEL}" \
      --config "${CONFIG_FILE}" \
      | tee "${CALIB_OUTPUT}"
    echo ""
    echo "Calibration written to ${CALIB_OUTPUT}"
    echo "Append its contents to ${CONFIG_FILE} before running damping or policy mode."
    ;;

  low-gain)
    verify_all
    # Pre-write low_gain_mode so the state machine enters LowGainTest directly
    # after motors are ready (no policy, reduced gains, static standing pose).
    cat >"${COMMAND_FILE}" <<EOF
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
clear_fault=false
low_gain_mode=true
EOF
    run_deploy --real --enable --clear-fault --policy-backend none
    echo "Low-gain mode exited. Use BACK button on joystick to toggle low_gain_mode."
    ;;

  *)
    echo "Unknown mode: ${MODE}" >&2
    usage >&2
    exit 1
    ;;
esac
