#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="$(cd "${ROOT}/.." && pwd)/.deps"
INSTALL_DIR="${DEPS_DIR}/openvino-2026.4.1"
VERSION="2026.4.1"
PACKAGE="openvino_toolkit_ubuntu22_${VERSION}.22982.07f9c262b05_x86_64"
ARCHIVE="/tmp/openvino_${VERSION}.tgz"

if [[ -f "${INSTALL_DIR}/setupvars.sh" ]]; then
  echo "OpenVINO ${VERSION} already installed at ${INSTALL_DIR}"
  exit 0
fi

mkdir -p "${DEPS_DIR}"
curl -L --fail --output "${ARCHIVE}" \
  "https://storage.openvinotoolkit.org/repositories/openvino/packages/${VERSION}/linux/${PACKAGE}.tgz"
mkdir -p "${INSTALL_DIR}"
tar -xzf "${ARCHIVE}" -C "${INSTALL_DIR}" --strip-components=1
echo "Installed OpenVINO ${VERSION} at ${INSTALL_DIR}"
