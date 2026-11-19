#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="$(cd "${ROOT}/.." && pwd)/.deps"
INSTALL_DIR="${DEPS_DIR}/opencv-4.12-openvino"
OPENVINO_DIR="${DEPS_DIR}/openvino-2026.4.1"
SOURCE_DIR="/tmp/opencv-4.12.0"
BUILD_DIR="/tmp/opencv-4.12-build"
VERSION="4.12.0"
ARCHIVE="/tmp/opencv-${VERSION}.tar.gz"

if [[ -f "${INSTALL_DIR}/lib/cmake/opencv4/OpenCVConfig.cmake" ]]; then
  echo "OpenCV ${VERSION} already installed at ${INSTALL_DIR}"
  exit 0
fi

if [[ ! -f "${OPENVINO_DIR}/runtime/cmake/OpenVINOConfig.cmake" ]]; then
  echo "OpenVINO 2026.4.1 is not installed; run tools/install_openvino.sh first" >&2
  exit 1
fi

if [[ ! -d "${SOURCE_DIR}" ]]; then
  curl -L --fail --output "${ARCHIVE}" \
    "https://github.com/opencv/opencv/archive/refs/tags/${VERSION}.tar.gz"
  tar -xzf "${ARCHIVE}" -C /tmp
fi

# OpenVINO 2026 returns const tensor data. OpenCV 4.12's legacy
# readNetFromModelOptimizer wrapper still constructs a non-const Mat view.
INF_ENGINE_SOURCE="${SOURCE_DIR}/modules/dnn/src/op_inf_engine.cpp"
if ! grep -q 'const_cast<void \*>(blob.data())' "${INF_ENGINE_SOURCE}"; then
  sed -i \
    's/return Mat(size, type, blob.data());/return Mat(size, type, const_cast<void *>(blob.data()));/' \
    "${INF_ENGINE_SOURCE}"
fi

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DBUILD_LIST=core,imgproc,imgcodecs,calib3d,dnn \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_JAVA=OFF \
  -DBUILD_opencv_python3=OFF \
  -DBUILD_PROTOBUF=ON \
  -DWITH_PROTOBUF=ON \
  -DWITH_OPENVINO=ON \
  -DOpenVINO_DIR="${OPENVINO_DIR}/runtime/cmake" \
  -DWITH_IPP=OFF \
  -DWITH_OPENCL=OFF \
  -DWITH_FFMPEG=OFF \
  -DWITH_V4L=OFF \
  -DWITH_GTK=OFF \
  -DWITH_QT=OFF \
  -DWITH_CUDA=OFF

cmake --build "${BUILD_DIR}" --parallel "$(nproc)"
cmake --install "${BUILD_DIR}"
