cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(ARMOR_DETECTOR_SOURCES
  src/detector.cpp
  src/detector_node.cpp
  src/number_classifier.cpp
  src/pnp_solver.cpp
)

# TensorRT + CUDA 是可选的。没有 TensorRT 时保留传统 CV 检测器，
# 使 x86 开发机和未安装 CUDA 的机器也能完整编译、运行。
find_library(NVINFER_LIB nvinfer)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin)
find_package(CUDA QUIET)
if(CUDA_FOUND)
  set(CUDART_LIB ${CUDA_LIBRARIES})
  set(CUDA_INCLUDE_DIR ${CUDA_INCLUDE_DIRS})
else()
  find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64)
  find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h PATHS /usr/local/cuda/include)
endif()

if(NVINFER_LIB AND NVINFER_PLUGIN_LIB AND CUDART_LIB AND CUDA_INCLUDE_DIR)
  message(STATUS "TensorRT support enabled")
  list(APPEND ARMOR_DETECTOR_SOURCES src/yolo_detector.cpp)
  set(TENSORRT_ENABLED TRUE)
else()
  message(STATUS "TensorRT/CUDA not found; using the classic CV detector fallback")
  list(APPEND ARMOR_DETECTOR_SOURCES src/yolo_detector_stub.cpp)
  set(TENSORRT_ENABLED FALSE)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED ${ARMOR_DETECTOR_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(TENSORRT_ENABLED)
  target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${NVINFER_LIB} ${NVINFER_PLUGIN_LIB} ${CUDART_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE RM_VISION_HAS_TENSORRT=1)
  # TensorRT 8.5 的 enqueueV2/destroy 等 API 已标记 deprecated，但仍可用。
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE RM_VISION_HAS_TENSORRT=0)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
