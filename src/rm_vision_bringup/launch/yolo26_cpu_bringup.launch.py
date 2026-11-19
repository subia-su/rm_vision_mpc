import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory


sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch"))


def generate_launch_description():
    from common import launch_params, node_params, robot_state_publisher, tracker_node
    from launch import LaunchDescription
    from launch.actions import Shutdown, TimerAction
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node

    if launch_params["camera"] != "hik":
        raise RuntimeError("YOLO26 DNN launch currently requires the hik camera")

    with open(node_params, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)
    camera_parameters = parameters["/camera_node"]["ros__parameters"]
    detector_parameters = parameters["/armor_detector"]["ros__parameters"]

    camera_detector_container = ComposableNodeContainer(
        name="camera_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_camera",
                plugin="hik_camera::HikCameraNode",
                name="camera_node",
                parameters=[camera_parameters],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="yolo26_dnn_detector",
                plugin="rm_auto_aim::Yolo26DnnNode",
                name="armor_detector",
                parameters=[detector_parameters],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        name="serial_driver",
        parameters=[node_params],
        output="both",
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    delay_serial = TimerAction(period=1.5, actions=[serial_driver_node])
    delay_tracker = TimerAction(period=2.0, actions=[tracker_node])
    return LaunchDescription(
        [robot_state_publisher, camera_detector_container, delay_serial, delay_tracker])
