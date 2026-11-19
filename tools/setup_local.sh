#!/usr/bin/env bash
# Source this file before building or running the workspace:
#   source tools/setup_local.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source tools/setup_local.sh 加载环境" >&2
  exit 1
fi

RM_VISION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RM_VISION_ROS_DISTRO="${ROS_DISTRO:-humble}"
RM_VISION_ROS_SETUP="/opt/ros/${RM_VISION_ROS_DISTRO}/setup.bash"

if [[ ! -f "${RM_VISION_ROS_SETUP}" ]]; then
  echo "未找到 ${RM_VISION_ROS_SETUP}，请先安装 ROS 2 ${RM_VISION_ROS_DISTRO}" >&2
  return 1
fi

source "${RM_VISION_ROS_SETUP}"

RM_VISION_DEPS_ROOT="$(cd "${RM_VISION_ROOT}/.." && pwd)/.deps"
RM_VISION_OPENVINO_DIR="${RM_VISION_DEPS_ROOT}/openvino-2026.4.1"
if [[ -f "${RM_VISION_OPENVINO_DIR}/setupvars.sh" ]]; then
  source "${RM_VISION_OPENVINO_DIR}/setupvars.sh" >/dev/null
fi

if [[ -f "${RM_VISION_ROOT}/install/setup.bash" ]]; then
  source "${RM_VISION_ROOT}/install/setup.bash"
fi

# 当前开发容器的 HOME 为只读；将 ROS 日志和缓存固定在可写工作区内，
# 同时避免不同终端混用旧的 DDS/日志状态。
export ROS_HOME="${RM_VISION_ROOT}/.ros"
export ROS_LOG_DIR="${RM_VISION_ROOT}/logs/ros"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RM_VISION_OPENVINO_DIR
export RM_VISION_OPENCV_DNN_DIR="${RM_VISION_DEPS_ROOT}/opencv-4.12-openvino"
mkdir -p "${ROS_LOG_DIR}"

echo "rm_vision 环境已加载: ${RM_VISION_ROOT} (${RM_VISION_ROS_DISTRO})"
