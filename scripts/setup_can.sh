#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${IFACE} ${BITRATE}" >&2
  exit 1
fi

ip link set down "${IFACE}" 2>/dev/null || true
ip link set "${IFACE}" type can bitrate "${BITRATE}" loopback off
ip link set up "${IFACE}"

echo "${IFACE} is up at ${BITRATE} bps"
ip -details link show "${IFACE}"
