cmake_minimum_required(VERSION 3.10)
project(yolo26_cpu_benchmark)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ENV{RM_VISION_OPENCV_DNN_DIR})
  message(FATAL_ERROR "Set RM_VISION_OPENCV_DNN_DIR or source tools/setup_local.sh")
endif()
set(RM_VISION_OPENCV_DNN_DIR "$ENV{RM_VISION_OPENCV_DNN_DIR}")
find_package(OpenCV 4.12 REQUIRED CONFIG
  PATHS "${RM_VISION_OPENCV_DNN_DIR}/lib/cmake/opencv4"
  NO_DEFAULT_PATH
)
find_package(OpenVINO REQUIRED)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(DETECTOR_DIR
  "${REPO_ROOT}/src/rm_auto_aim/yolo26_dnn_detector")

add_executable(yolo26_cpu_benchmark
  benchmark.cpp
  "${DETECTOR_DIR}/src/dnn_preprocess.cpp"
  "${DETECTOR_DIR}/src/openvino_yolo_detector.cpp"
  "${DETECTOR_DIR}/src/opencv_yolo_detector.cpp"
)
target_include_directories(yolo26_cpu_benchmark PRIVATE
  "${DETECTOR_DIR}/include"
)
target_link_libraries(yolo26_cpu_benchmark ${OpenCV_LIBS} openvino::runtime)
set_target_properties(yolo26_cpu_benchmark PROPERTIES
  BUILD_RPATH "${RM_VISION_OPENCV_DNN_DIR}/lib"
)
