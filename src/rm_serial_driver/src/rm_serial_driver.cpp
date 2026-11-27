// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/packet_stream.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "rm_serial_driver/trajectory_packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  mpc_horizon_ = static_cast<std::size_t>(declare_parameter<int>("mpc.horizon", 100));
  mpc_dt_ = declare_parameter("mpc.dt", 0.01);
  bullet_speed_ = declare_parameter("aim.bullet_speed", 25.0);
  system_delay_ = declare_parameter("aim.system_delay", 0.03);
  gravity_ = declare_parameter("aim.gravity", 9.80665);
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count("mpc.pitch_min") || overrides.count("mpc.pitch_max")) {
    throw std::invalid_argument(
      "Remove legacy mechanical mpc.pitch_min/max; use trajectory.world_pitch_min/max in world radians");
  }
  const auto pitch_frame = declare_parameter<std::string>("trajectory.pitch_frame", "world");
  trajectory_policy_.ttl_ms = declare_parameter<int>("trajectory.ttl_ms", 40);
  trajectory_policy_.transport_age_ms = declare_parameter<int>("trajectory.transport_age_ms", 2);
  trajectory_policy_.pitch_min = declare_parameter("trajectory.world_pitch_min", -std::acos(-1.0) / 2.0);
  trajectory_policy_.pitch_max = declare_parameter("trajectory.world_pitch_max", std::acos(-1.0) / 2.0);
  trajectory_policy_.target_timeout_ms = declare_parameter("aim.target_timeout_ms", 50.0);
  MpcLimits limits;
  limits.yaw_speed = declare_parameter("mpc.yaw_speed_limit", 5.0);
  limits.pitch_speed = declare_parameter("mpc.pitch_speed_limit", 0.6);
  limits.yaw_acc = declare_parameter("mpc.yaw_acc_limit", 20.0);
  limits.pitch_acc = declare_parameter("mpc.pitch_acc_limit", 8.0);
  MpcWeights weights;
  weights.angle = declare_parameter("mpc.angle_weight", 30.0);
  weights.speed = declare_parameter("mpc.speed_weight", 0.8);
  weights.acceleration = declare_parameter("mpc.acceleration_weight", 0.04);
  weights.acceleration_delta = declare_parameter("mpc.jerk_weight", 0.2);
  trajectory_policy_.limits = limits;
  validate_world_policy(trajectory_policy_, pitch_frame);
  if (!std::isfinite(bullet_speed_) || bullet_speed_ <= 1.0 ||
    !std::isfinite(system_delay_) || system_delay_ < 0.0 ||
    !std::isfinite(gravity_) || gravity_ <= 0.0)
  {
    throw std::invalid_argument("invalid ballistic or A6 freshness parameters");
  }
  mpc_ = std::make_unique<MpcPlanner>(mpc_horizon_, mpc_dt_, limits, weights);

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  trajectory_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/serial_driver/trajectory", 10);
  planner_status_pub_ = this->create_publisher<std_msgs::msg::String>("/serial_driver/planner_status", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  shutting_down_.store(true, std::memory_order_release);
  {
    std::lock_guard<std::mutex> reconnect_lock(reopen_mutex_);
    // Closing the port releases receive() so the worker can observe shutdown.
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> data(64);
  ReceivePacketParser parser;

  while (rclcpp::ok()) {
    const auto observed_revision = port_revision_.load(std::memory_order_acquire);
    try {
      // read_some may return a fragment, one packet, or several packets.
      const auto received = serial_driver_->port()->receive(data);
      for (std::size_t i = 0; i < received; ++i) {
        ReceivePacket packet{};
        if (parser.push(data[i], packet)) {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(
            from_fixed16(packet.roll), from_fixed16(packet.pitch), from_fixed16(packet.yaw));
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          if (std::abs(from_fixed16(packet.aim_x)) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = from_fixed16(packet.aim_x);
            aiming_point_.pose.position.y = from_fixed16(packet.aim_y);
            aiming_point_.pose.position.z = from_fixed16(packet.aim_z);
            marker_pub_->publish(aiming_point_);
          }
        }
      }
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "Feedback frames=%u, CRC errors=%u",
        parser.frames(), parser.crc_errors());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      parser.reset();
      reopenPort(observed_revision);
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const auto observed_revision = port_revision_.load(std::memory_order_acquire);
  try {
    const auto reference_origin = std::chrono::steady_clock::now();
    TrajectoryCommand command;
    std::string planner_status = "no_tracking_target";
    command.sequence = ++trajectory_sequence_;
    command.host_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000LL);
    command.ttl_ms = static_cast<uint16_t>(trajectory_policy_.ttl_ms);
    const double message_age = (this->now() - msg->header.stamp).seconds();
    try {
      if (msg->tracking &&
        (msg->armors_num == 2 || msg->armors_num == 3 || msg->armors_num == 4))
      {
        const auto age_error = target_age_error(message_age, trajectory_policy_);
        if (!age_error.empty()) {throw std::domain_error(age_error);}
        std::vector<AimReference> reference;
        reference.reserve(mpc_horizon_);
        for (std::size_t k = 0; k < mpc_horizon_; ++k) {
          double flight = 0.0, x = 0.0, y = 0.0, z = 0.0;
          for (unsigned iteration = 0; iteration < 3; ++iteration) {
            const double t = message_age + system_delay_ +
              (static_cast<double>(k) - static_cast<double>(mpc_horizon_ / 2)) * mpc_dt_ + flight;
            const double center_x = msg->position.x + msg->velocity.x * t;
            const double center_y = msg->position.y + msg->velocity.y * t;
            const double center_z = msg->position.z + msg->velocity.z * t;
            double best_distance = std::numeric_limits<double>::infinity();
            for (int armor = 0; armor < msg->armors_num; ++armor) {
              const double armor_yaw = msg->yaw + msg->v_yaw * t +
                armor * 2.0 * std::acos(-1.0) / msg->armors_num;
              const bool alternate = msg->armors_num == 4 && (armor & 1);
              const double radius = alternate ? msg->radius_2 : msg->radius_1;
              const double candidate_x = center_x - radius * std::cos(armor_yaw);
              const double candidate_y = center_y - radius * std::sin(armor_yaw);
              const double candidate_z = center_z + (alternate ? msg->dz : 0.0);
              const double distance = candidate_x * candidate_x + candidate_y * candidate_y +
                candidate_z * candidate_z;
              if (distance < best_distance) {
                best_distance = distance;
                x = candidate_x; y = candidate_y; z = candidate_z;
              }
            }
            flight = std::hypot(std::hypot(x, y), z) / bullet_speed_;
          }
          const double horizontal = std::hypot(x, y);
          if (horizontal < 0.05) throw std::domain_error("target is too close");
          const double v2 = bullet_speed_ * bullet_speed_;
          const double discriminant = v2 * v2 - gravity_ *
            (gravity_ * horizontal * horizontal + 2.0 * z * v2);
          if (!std::isfinite(discriminant) || discriminant <= 0.0) {
            throw std::domain_error("target has no ballistic solution");
          }
          const double elevation =
            std::atan((v2 - std::sqrt(discriminant)) / (gravity_ * horizontal));
          reference.push_back({std::atan2(y, x), -elevation});
        }
        const PlannedState plan = mpc_->solve(reference);
        command.valid = true;
        command.yaw = static_cast<float>(plan.yaw);
        command.pitch = static_cast<float>(plan.pitch);
        command.yaw_speed = static_cast<float>(plan.yaw_speed);
        command.pitch_speed = static_cast<float>(plan.pitch_speed);
        command.yaw_acc = static_cast<float>(plan.yaw_acc);
        command.pitch_acc = static_cast<float>(plan.pitch_acc);
        planner_status = "ok_world_pitch";
      } else {
        mpc_->reset();
        if (msg->tracking) {planner_status = "invalid_armor_count";}
      }
    } catch (const std::logic_error & ex) {
      // Always send a newer valid=0 to revoke the MCU's previous reference.
      revoke_trajectory(command);
      mpc_->reset();
      planner_status = ex.what();
    }
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - reference_origin).count();
    const auto contract_error = finalize_world_trajectory(command, trajectory_policy_, elapsed_ms);
    if (!contract_error.empty()) {planner_status = contract_error; mpc_->reset();}
    const auto packed = pack_trajectory(command);
    const std::vector<uint8_t> data(packed.begin(), packed.end());

    sendPacketFully(data, [this](const std::vector<uint8_t> & remaining) {
      return serial_driver_->port()->send(remaining);
    });

    std_msgs::msg::Float64MultiArray trajectory_debug;
    trajectory_debug.data = {
      command.valid ? 1.0 : 0.0,
      static_cast<double>(command.sequence),
      static_cast<double>(command.host_ms),
      static_cast<double>(command.age_ms),
      static_cast<double>(command.ttl_ms),
      static_cast<double>(command.yaw),
      static_cast<double>(command.pitch),
      static_cast<double>(command.yaw_speed),
      static_cast<double>(command.pitch_speed),
      static_cast<double>(command.yaw_acc),
      static_cast<double>(command.pitch_acc),
    };
    trajectory_pub_->publish(trajectory_debug);
    std_msgs::msg::String status_message;
    status_message.data = planner_status;
    planner_status_pub_->publish(status_message);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::logic_error & ex) {
    // Errors outside planning/contract validation still leave MCU expiry active.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Rejected target packet: %s", ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort(observed_revision);
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort(std::uint64_t observed_revision)
{
  std::unique_lock<std::mutex> reconnect_lock(reopen_mutex_);
  if (
    shutting_down_.load(std::memory_order_acquire) ||
    port_revision_.load(std::memory_order_acquire) != observed_revision)
  {
    return;
  }

  while (rclcpp::ok() && !shutting_down_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try {
      if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      port_revision_.fetch_add(1, std::memory_order_release);
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
      return;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
