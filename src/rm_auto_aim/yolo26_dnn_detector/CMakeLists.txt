cmake_minimum_required(VERSION 3.10)
project(yolo26_dnn_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(OpenVINO REQUIRED)

if(NOT DEFINED ENV{RM_VISION_OPENCV_DNN_DIR})
  message(FATAL_ERROR
    "RM_VISION_OPENCV_DNN_DIR is not set. Source tools/setup_local.sh before building.")
endif()
set(RM_VISION_OPENCV_DNN_DIR "$ENV{RM_VISION_OPENCV_DNN_DIR}")
find_package(OpenCV 4.12 REQUIRED CONFIG
  PATHS "${RM_VISION_OPENCV_DNN_DIR}/lib/cmake/opencv4"
  NO_DEFAULT_PATH
)
message(STATUS "YOLO26 CPU DNN uses OpenCV ${OpenCV_VERSION}")

add_library(yolo26_dnn_node_component SHARED
  src/openvino_yolo_detector.cpp
  src/opencv_yolo_detector.cpp
  src/yolo26_dnn_node.cpp
)
target_include_directories(yolo26_dnn_node_component PRIVATE include)
target_link_libraries(yolo26_dnn_node_component ${OpenCV_LIBS} openvino::runtime)
ament_target_dependencies(yolo26_dnn_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  ament_index_cpp
  auto_aim_interfaces
)

set_target_properties(yolo26_dnn_node_component PROPERTIES
  BUILD_RPATH "${RM_VISION_OPENCV_DNN_DIR}/lib;$ENV{RM_VISION_OPENVINO_DIR}/runtime/lib/intel64"
  INSTALL_RPATH "${RM_VISION_OPENCV_DNN_DIR}/lib;$ENV{RM_VISION_OPENVINO_DIR}/runtime/lib/intel64"
)

rclcpp_components_register_node(
  yolo26_dnn_node_component
  PLUGIN "rm_auto_aim::Yolo26DnnNode"
  EXECUTABLE yolo26_dnn_node
)

install(TARGETS yolo26_dnn_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS yolo26_dnn_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
