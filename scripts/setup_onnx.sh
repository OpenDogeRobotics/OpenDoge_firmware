#!/usr/bin/env bash
set -euo pipefail

# Download and install ONNX Runtime for OpenDoge deployment.
# Places headers and libs under build/deps/onnxruntime/ (gitignored).
#
# Usage:
#   ./scripts/setup_onnx.sh                  # default version and path
#   ONNX_VERSION=1.20.1 ./scripts/setup_onnx.sh
#   DEPS_DIR=~/onnx ./scripts/setup_onnx.sh
#
# After setup, build with:
#   ONNXRUNTIME_ROOT=build/deps/onnxruntime colcon build ...

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DEPS_DIR="${DEPS_DIR:-${PROJECT_DIR}/build/deps}"
ONNX_VERSION="${ONNX_VERSION:-1.18.1}"
INSTALL_DIR="${DEPS_DIR}/onnxruntime-${ONNX_VERSION}"
SYMLINK_DIR="${DEPS_DIR}/onnxruntime"

# --- detect platform ---
ARCH="$(uname -m)"
case "$ARCH" in
  x86_64)  ONNX_ARCH="x64" ;;
  aarch64) ONNX_ARCH="arm64" ;;
  *)
    echo "Error: unsupported architecture: $ARCH"
    exit 1
    ;;
esac

TGZ="onnxruntime-linux-${ONNX_ARCH}-${ONNX_VERSION}.tgz"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${TGZ}"

echo "=== OpenDoge ONNX Runtime Setup ==="
echo "  Version:    ${ONNX_VERSION}"
echo "  Arch:       ${ONNX_ARCH}"
echo "  Install to: ${INSTALL_DIR}"
echo "  Symlink:    ${SYMLINK_DIR} -> ${INSTALL_DIR}"
echo ""

# --- already installed? ---
if [ -f "${INSTALL_DIR}/include/onnxruntime_cxx_api.h" ] && \
   [ -f "${INSTALL_DIR}/lib/libonnxruntime.so" ]; then
  echo "Already installed at ${INSTALL_DIR}"
else
  mkdir -p "${DEPS_DIR}"

  TGZ_PATH="${DEPS_DIR}/${TGZ}"
  if [ ! -f "${TGZ_PATH}" ]; then
    echo "Downloading ${URL} ..."
    if command -v wget &>/dev/null; then
      wget -q --show-progress -O "${TGZ_PATH}" "${URL}"
    elif command -v curl &>/dev/null; then
      curl -L --progress-bar -o "${TGZ_PATH}" "${URL}"
    else
      echo "Error: need wget or curl to download"
      exit 1
    fi
  else
    echo "Using cached ${TGZ_PATH}"
  fi

  echo "Extracting to ${INSTALL_DIR} ..."
  rm -rf "${INSTALL_DIR}"
  mkdir -p "${INSTALL_DIR}"
  tar xzf "${TGZ_PATH}" -C "${INSTALL_DIR}" --strip-components=1

  # Remove unused files (training libs, debug symbols, etc.)
  rm -f "${INSTALL_DIR}"/lib/libonnxruntime.so.*.dbg 2>/dev/null || true
  rm -f "${INSTALL_DIR}"/lib/libonnxruntime_providers_*.so 2>/dev/null || true

  echo "Installed to ${INSTALL_DIR}"
fi

# --- update symlink ---
if [ -L "${SYMLINK_DIR}" ] || [ ! -e "${SYMLINK_DIR}" ]; then
  ln -sfn "${INSTALL_DIR}" "${SYMLINK_DIR}"
else
  echo "Warning: ${SYMLINK_DIR} exists and is not a symlink; skipping symlink"
fi

# --- print summary ---
echo ""
echo "=== Setup complete ==="
echo ""
echo "Add to your shell rc or run before build:"
echo "  export ONNXRUNTIME_ROOT=${SYMLINK_DIR}"
echo ""
echo "Then build:"
echo "  cd ${PROJECT_DIR}"
echo "  ONNXRUNTIME_ROOT=${SYMLINK_DIR} colcon build --symlink-install --packages-select opendoge_deploy"
echo ""
echo "Verify:"
echo "  ldd install/opendoge_deploy/bin/opendoge_deploy | grep onnx"
echo "  install/opendoge_deploy/bin/opendoge_deploy --policy-backend onnx --help"
