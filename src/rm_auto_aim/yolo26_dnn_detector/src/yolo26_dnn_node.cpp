// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/opencv_yolo_detector.hpp"
#include "yolo26_dnn_detector/openvino_yolo_detector.hpp"
#include "yolo26_dnn_detector/pnp_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/yolo_boxes.hpp"

namespace rm_auto_aim
{
namespace
{
struct ClassMetadata
{
  std::string number;
  std::string armor_type;
  bool large;
};

ClassMetadata classMetadata(const std::string & class_name)
{
  const auto separator = class_name.find('_');
  const std::string role =
    separator == std::string::npos ? class_name : class_name.substr(separator + 1);

  if (role == "sentry") {
    return {"guard", "large", true};
  }
  if (role == "hero") {
    return {"1", "small", false};
  }
  if (role == "engineer") {
    return {"2", "small", false};
  }
  if (role.rfind("infantry_", 0) == 0) {
    return {role.substr(9), "small", false};
  }
  if (role == "outpost") {
    return {"outpost", "small", false};
  }
  if (role == "base_large") {
    return {"base", "large", true};
  }
  if (role == "base_small") {
    return {"base", "small", false};
  }
  return {"", "small", false};
}

std::vector<cv::Point3f> objectPoints(bool large)
{
  const double width = (large ? 225.0 : 135.0) / 1000.0;
  const double height = 55.0 / 1000.0;
  return {
    {0.0F, static_cast<float>(width / 2), static_cast<float>(-height / 2)},
    {0.0F, static_cast<float>(width / 2), static_cast<float>(height / 2)},
    {0.0F, static_cast<float>(-width / 2), static_cast<float>(height / 2)},
    {0.0F, static_cast<float>(-width / 2), static_cast<float>(-height / 2)},
  };
}

std::array<double, 4> rotationToQuaternion(const cv::Mat & rotation)
{
  const double trace =
    rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2);
  double w, x, y, z;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / s;
    y = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / s;
    z = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / s;
  } else if (
    rotation.at<double>(0, 0) > rotation.at<double>(1, 1) &&
    rotation.at<double>(0, 0) > rotation.at<double>(2, 2))
  {
    const double s =
      std::sqrt(1.0 + rotation.at<double>(0, 0) - rotation.at<double>(1, 1) -
      rotation.at<double>(2, 2)) * 2.0;
    w = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / s;
    x = 0.25 * s;
    y = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / s;
    z = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / s;
  } else if (rotation.at<double>(1, 1) > rotation.at<double>(2, 2)) {
    const double s =
      std::sqrt(1.0 + rotation.at<double>(1, 1) - rotation.at<double>(0, 0) -
      rotation.at<double>(2, 2)) * 2.0;
    w = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / s;
    x = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / s;
    y = 0.25 * s;
    z = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / s;
  } else {
    const double s =
      std::sqrt(1.0 + rotation.at<double>(2, 2) - rotation.at<double>(0, 0) -
      rotation.at<double>(1, 1)) * 2.0;
    w = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / s;
    x = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / s;
    y = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / s;
    z = 0.25 * s;
  }
  return {x, y, z, w};
}

}  // namespace

class Yolo26DnnNode : public rclcpp::Node
{
public:
  explicit Yolo26DnnNode(const rclcpp::NodeOptions & options)
  : Node("armor_detector", options)
  {
    const std::string armor_detector_share =
      ament_index_cpp::get_package_share_directory("armor_detector");
    const std::string backend = this->declare_parameter<std::string>(
      "dnn.backend", "opencv_dnn");
    const bool use_openvino = backend == "openvino_fp32" || backend == "openvino_int8";
    const std::string default_model =
      backend == "openvino_int8" ?
      armor_detector_share + "/model/yolo26_obb_int8.xml" :
      use_openvino ?
      armor_detector_share + "/model/yolo26_obb_fp32.xml" :
      armor_detector_share + "/model/yolo26_obb.onnx";
    const std::string default_classes =
      armor_detector_share + "/model/yolo26_classes.txt";
    const std::string model_path =
      this->declare_parameter("dnn.model_path", default_model);
    std::string default_bin = model_path;
    const auto extension = default_bin.rfind(".xml");
    if (extension != std::string::npos) {
      default_bin.replace(extension, 4, ".bin");
    }
    const std::string model_bin =
      this->declare_parameter("dnn.model_bin", use_openvino ? default_bin : "");
    const std::string classes_path =
      this->declare_parameter("dnn.classes_path", default_classes);
    const float confidence =
      this->declare_parameter("dnn.conf_threshold", 0.25F);
    const float nms = this->declare_parameter("dnn.nms_threshold", 0.5F);
    const int threads = this->declare_parameter("dnn.threads", 16);
    this->declare_parameter("detect_color", 0);

    PnpFilterConfig filter_config;
    filter_config.max_reprojection_error_px =
      this->declare_parameter("filter.max_reprojection_error_px", 8.0);
    filter_config.min_box_aspect =
      this->declare_parameter("filter.min_box_aspect", 1.2);
    filter_config.max_box_aspect =
      this->declare_parameter("filter.max_box_aspect", 6.5);
    filter_config.min_depth_m =
      this->declare_parameter("filter.min_depth_m", 0.05);
    filter_config.max_depth_m =
      this->declare_parameter("filter.max_depth_m", 12.0);
    filter_config.temporal_gate_seconds =
      this->declare_parameter("filter.temporal_gate_seconds", 0.15);
    filter_config.max_position_jump_m =
      this->declare_parameter("filter.max_position_jump_m", 1.5);
    filter_config.max_image_jump_px =
      this->declare_parameter("filter.max_image_jump_px", 220.0);
    filter_config.max_angle_jump_rad =
      this->declare_parameter("filter.max_angle_jump_rad", 1.4);
    pnp_filter_ = std::make_unique<PnpFilter>(filter_config);

    if (use_openvino) {
      detector_ = std::make_unique<OpenVinoYoloDetector>(
        model_path, classes_path, confidence, nms, threads);
    } else {
      detector_ = std::make_unique<OpenCvYoloDetector>(
        model_path, classes_path, confidence, nms, threads, model_bin, false);
    }

    armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS());
    yolo_boxes_pub_ = this->create_publisher<auto_aim_interfaces::msg::YoloBoxes>(
      "/detector/yolo_boxes", 10);
    camera_info_sub_ =
      this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            camera_matrix_.at<double>(row, column) = message->k[row * 3 + column];
          }
        }
        distortion_.create(1, static_cast<int>(message->d.size()), CV_64F);
        for (size_t index = 0; index < message->d.size(); ++index) {
          distortion_.at<double>(0, static_cast<int>(index)) = message->d[index];
        }
      });

    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(1);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", image_qos,
      std::bind(&Yolo26DnnNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "YOLO26 DNN started: backend=%s model=%s threads=%d",
      backend.c_str(), model_path.c_str(), threads);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    if (!camera_matrix_.empty()) {
      const bool is_rgb = message->encoding == "rgb8";
      const cv::Mat image(
        static_cast<int>(message->height), static_cast<int>(message->width),
        CV_8UC3, const_cast<uint8_t *>(message->data.data()), message->step);
      const auto detections = detector_->detect(
        image, is_rgb, static_cast<int>(this->get_parameter("detect_color").as_int()));

      auto_aim_interfaces::msg::Armors armors;
      auto_aim_interfaces::msg::YoloBoxes boxes;
      armors.header = message->header;
      boxes.header = message->header;
      for (const auto & detection : detections) {
        const std::string & class_name = detector_->className(detection.class_id);
        const ClassMetadata metadata = classMetadata(class_name);
        if (metadata.number.empty()) {
          continue;
        }

        std::vector<cv::Point2f> image_points;
        for (const auto & corner : detection.corners) {
          image_points.emplace_back(corner.x, corner.y);
        }
        cv::Mat rotation_vector;
        cv::Mat translation_vector;
        const auto object_points = objectPoints(metadata.large);
        if (
          !cv::solvePnP(
            object_points, image_points, camera_matrix_, distortion_,
            rotation_vector, translation_vector, false, cv::SOLVEPNP_IPPE))
        {
          continue;
        }

        const double stamp_seconds = rclcpp::Time(message->header.stamp).seconds();
        const PnpFilterResult filter_result = pnp_filter_->evaluate(
          metadata.number, stamp_seconds, detection.width, detection.height,
          cv::Point2f(detection.center_x, detection.center_y), detection.angle,
          rotation_vector, translation_vector, camera_matrix_, distortion_,
          object_points, image_points);
        if (!filter_result.accepted) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Rejected PnP result (%s): %s reproj=%.2fpx jump=%.2fm %.1fpx %.2frad",
            metadata.number.c_str(), filter_result.reason.c_str(),
            filter_result.reprojection_error_px, filter_result.position_jump_m,
            filter_result.image_jump_px, filter_result.angle_jump_rad);
          continue;
        }

        cv::Mat rotation_matrix;
        cv::Rodrigues(rotation_vector, rotation_matrix);
        const auto quaternion = rotationToQuaternion(rotation_matrix);

        auto_aim_interfaces::msg::Armor armor;
        armor.number = metadata.number;
        armor.type = metadata.armor_type;
        armor.confidence = detection.score;
        armor.reprojection_error = static_cast<float>(
          filter_result.reprojection_error_px);
        armor.pose.position.x = translation_vector.at<double>(0);
        armor.pose.position.y = translation_vector.at<double>(1);
        armor.pose.position.z = translation_vector.at<double>(2);
        armor.pose.orientation.x = quaternion[0];
        armor.pose.orientation.y = quaternion[1];
        armor.pose.orientation.z = quaternion[2];
        armor.pose.orientation.w = quaternion[3];
        armor.distance_to_image_center = static_cast<float>(
          cv::norm(cv::Point2f(detection.center_x, detection.center_y) - cv::Point2f(
            camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2))));
        armors.armors.push_back(armor);

        auto_aim_interfaces::msg::YoloBox box;
        box.class_name = class_name;
        box.number = metadata.number;
        box.color = class_name.rfind("red_", 0) == 0 ? "RED" : "BLUE";
        box.confidence = detection.score;
        for (size_t index = 0; index < detection.corners.size(); ++index) {
          box.corners[index].x = detection.corners[index].x;
          box.corners[index].y = detection.corners[index].y;
        }
        boxes.boxes.push_back(box);
      }
      armors_pub_->publish(armors);
      yolo_boxes_pub_->publish(boxes);
    }
  }

  std::unique_ptr<YoloDetectorBase> detector_;
  std::unique_ptr<PnpFilter> pnp_filter_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::YoloBoxes>::SharedPtr yolo_boxes_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
};

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Yolo26DnnNode)
