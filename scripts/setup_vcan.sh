#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${IFACE}" >&2
  exit 1
fi

modprobe vcan
ip link del "${IFACE}" 2>/dev/null || true
ip link add dev "${IFACE}" type vcan
ip link set up "${IFACE}"

echo "${IFACE} vcan is up"
ip -details link show "${IFACE}"
