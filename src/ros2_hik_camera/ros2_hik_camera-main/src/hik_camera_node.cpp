#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // Init convert param
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    nRet = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (MV_OK != nRet) {
      throw std::runtime_error("Failed to read camera image info after ROI configuration");
    }
    image_msg_.data.reserve(
      static_cast<size_t>(img_info_.nHeightMax) * img_info_.nWidthMax * 3);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    if (
      camera_info_msg_.width != img_info_.nWidthValue ||
      camera_info_msg_.height != img_info_.nHeightValue)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Camera info is %ux%u but camera output is %ux%u; recalibrate or fix ROI",
        camera_info_msg_.width, camera_info_msg_.height, img_info_.nWidthValue,
        img_info_.nHeightValue);
    }

    MV_CC_StartGrabbing(camera_handle_);

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = image_msg_.width * 3;
          image_msg_.data.resize(static_cast<size_t>(image_msg_.step) * image_msg_.height);

          // MV_CC_ConvertPixelType writes into a preallocated destination buffer.
          // resize() must happen before assigning pDstBuffer, because reserve() does
          // not change vector::size().
          convert_param_.nWidth = image_msg_.width;
          convert_param_.nHeight = image_msg_.height;
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          const int convert_ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
          if (MV_OK != convert_ret) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Pixel conversion failed! nRet: [%x]", convert_ret);
            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            continue;
          }

          image_msg_.header.stamp = this->now();

          camera_info_msg_.header = image_msg_.header;
          camera_info_msg_.width = image_msg_.width;
          camera_info_msg_.height = image_msg_.height;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_count_++;
        }

        if (fail_count_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;

    int width = this->declare_parameter("width", static_cast<int>(img_info_.nWidthValue));
    int height = this->declare_parameter("height", static_cast<int>(img_info_.nHeightValue));
    int offset_x = this->declare_parameter("offset_x", -1);
    int offset_y = this->declare_parameter("offset_y", -1);
    int sensor_width = this->declare_parameter(
      "sensor_width", static_cast<int>(img_info_.nWidthMax));
    int sensor_height = this->declare_parameter(
      "sensor_height", static_cast<int>(img_info_.nHeightMax));
    configureRoi(width, height, offset_x, offset_y, sensor_width, sensor_height);

    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    double frame_rate = this->declare_parameter("frame_rate", static_cast<double>(f_value.fCurValue));
    if (frame_rate <= 0.0) {
      throw std::invalid_argument("frame_rate must be greater than zero");
    }
    int ret = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    if (MV_OK != ret) {
      throw std::runtime_error(
              "Failed to enable AcquisitionFrameRate, status = " + std::to_string(ret));
    }
    ret = MV_CC_SetFloatValue(
      camera_handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate));
    if (MV_OK != ret) {
      throw std::runtime_error(
              "Failed to set AcquisitionFrameRate, status = " + std::to_string(ret));
    }
    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f", f_value.fCurValue);

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  void configureRoi(
    int width, int height, int requested_offset_x, int requested_offset_y,
    int sensor_width, int sensor_height)
  {
    if (
      sensor_width <= 0 || sensor_height <= 0 || width > sensor_width ||
      height > sensor_height)
    {
      throw std::invalid_argument("ROI must fit within the configured sensor dimensions");
    }

    MVCC_INTVALUE_EX width_range{};
    MVCC_INTVALUE_EX height_range{};
    MVCC_INTVALUE_EX offset_x_range{};
    MVCC_INTVALUE_EX offset_y_range{};

    auto get_range = [this](const char * feature, MVCC_INTVALUE_EX & range) {
        const int ret = MV_CC_GetIntValueEx(camera_handle_, feature, &range);
        if (MV_OK != ret) {
          throw std::runtime_error(
                  "Failed to read camera feature " + std::string(feature) +
                  ", status = " + std::to_string(ret));
        }
      };
    auto align_down = [](int64_t value, int64_t increment) {
        if (increment <= 1) {
          return value;
        }
        return value - value % increment;
      };
    auto validate = [](int64_t value, const MVCC_INTVALUE_EX & range, const char * feature) {
        if (
          value < range.nMin || value > range.nMax ||
          (range.nInc > 1 && value % range.nInc != 0))
        {
          throw std::invalid_argument(
                  std::string(feature) + " must be within [" +
                  std::to_string(range.nMin) + ", " + std::to_string(range.nMax) +
                  "] and a multiple of " + std::to_string(range.nInc));
        }
      };

    get_range("Width", width_range);
    get_range("Height", height_range);
    // Width/Height maxima can depend on the previous offsets. Refresh all
    // ranges after resetting offsets and applying the requested dimensions.
    get_range("Width", width_range);
    get_range("Height", height_range);
    get_range("OffsetX", offset_x_range);
    get_range("OffsetY", offset_y_range);

    const int64_t aligned_width = align_down(width, width_range.nInc);
    const int64_t aligned_height = align_down(height, height_range.nInc);
    validate(aligned_width, width_range, "width");
    validate(aligned_height, height_range, "height");

    // Start from offset zero so the camera can reduce Width/Height first.
    int ret = MV_CC_SetIntValueEx(camera_handle_, "OffsetX", 0);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to reset OffsetX, status = " + std::to_string(ret));
    }
    ret = MV_CC_SetIntValueEx(camera_handle_, "OffsetY", 0);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to reset OffsetY, status = " + std::to_string(ret));
    }

    // Some Hikrobot models persist ROI settings across processes and report a
    // reduced feature maximum until the sensor is restored to full frame.
    ret = MV_CC_SetIntValueEx(camera_handle_, "Width", sensor_width);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to restore sensor width, status = " + std::to_string(ret));
    }
    ret = MV_CC_SetIntValueEx(camera_handle_, "Height", sensor_height);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to restore sensor height, status = " + std::to_string(ret));
    }

    ret = MV_CC_SetIntValueEx(camera_handle_, "Width", aligned_width);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to set Width, status = " + std::to_string(ret));
    }
    ret = MV_CC_SetIntValueEx(camera_handle_, "Height", aligned_height);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to set Height, status = " + std::to_string(ret));
    }

    get_range("OffsetX", offset_x_range);
    get_range("OffsetY", offset_y_range);
    const int64_t max_offset_x = sensor_width - aligned_width;
    const int64_t max_offset_y = sensor_height - aligned_height;
    const int64_t center_x = align_down(max_offset_x / 2, offset_x_range.nInc);
    const int64_t center_y = align_down(max_offset_y / 2, offset_y_range.nInc);
    const int64_t final_offset_x = requested_offset_x < 0 ? center_x : requested_offset_x;
    const int64_t final_offset_y = requested_offset_y < 0 ? center_y : requested_offset_y;
    if (
      final_offset_x < 0 || final_offset_x > max_offset_x ||
      (offset_x_range.nInc > 1 && final_offset_x % offset_x_range.nInc != 0))
    {
      throw std::invalid_argument(
              "offset_x must be within [0, " + std::to_string(max_offset_x) +
              "] and a multiple of " + std::to_string(offset_x_range.nInc));
    }
    if (
      final_offset_y < 0 || final_offset_y > max_offset_y ||
      (offset_y_range.nInc > 1 && final_offset_y % offset_y_range.nInc != 0))
    {
      throw std::invalid_argument(
              "offset_y must be within [0, " + std::to_string(max_offset_y) +
              "] and a multiple of " + std::to_string(offset_y_range.nInc));
    }

    ret = MV_CC_SetIntValueEx(camera_handle_, "OffsetX", final_offset_x);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to set OffsetX, status = " + std::to_string(ret));
    }
    ret = MV_CC_SetIntValueEx(camera_handle_, "OffsetY", final_offset_y);
    if (MV_OK != ret) {
      throw std::runtime_error("Failed to set OffsetY, status = " + std::to_string(ret));
    }

    MVCC_INTVALUE_EX actual_width{};
    MVCC_INTVALUE_EX actual_height{};
    MVCC_INTVALUE_EX actual_offset_x{};
    MVCC_INTVALUE_EX actual_offset_y{};
    get_range("Width", actual_width);
    get_range("Height", actual_height);
    get_range("OffsetX", actual_offset_x);
    get_range("OffsetY", actual_offset_y);
    RCLCPP_INFO(
      this->get_logger(), "ROI: %ldx%ld offset=(%ld,%ld)",
      actual_width.nCurValue, actual_height.nCurValue, actual_offset_x.nCurValue,
      actual_offset_y.nCurValue);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
