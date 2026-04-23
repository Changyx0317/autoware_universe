// Copyright 2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/trajectory_follower_node/controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"
#include "autoware_utils/ros/marker_helper.hpp"

#include <autoware/trajectory_follower_base/lateral_controller_base.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}
}  // namespace

namespace autoware::motion::control::trajectory_follower_node
{
Controller::Controller(const rclcpp::NodeOptions & node_options) : Node("controller", node_options)
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period");
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec");
  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);
  parking_three_phase_param_.enable =
    declare_parameter<bool>("parking_three_phase.enable", true);
  parking_three_phase_param_.stop_velocity_threshold =
    declare_parameter<double>("parking_three_phase.stop_velocity_threshold_mps", 0.05);
  parking_three_phase_param_.pre_align_enter_yaw_threshold =
    declare_parameter<double>("parking_three_phase.pre_align_enter_yaw_threshold_rad", 0.10);
  parking_three_phase_param_.pre_align_exit_yaw_threshold =
    declare_parameter<double>("parking_three_phase.pre_align_exit_yaw_threshold_rad", 0.05);
  parking_three_phase_param_.pre_align_min_target_distance =
    declare_parameter<double>("parking_three_phase.pre_align_min_target_distance_m", 0.30);
  parking_three_phase_param_.post_align_enter_distance =
    declare_parameter<double>("parking_three_phase.post_align_enter_distance_m", 0.35);
  parking_three_phase_param_.post_align_keep_distance =
    declare_parameter<double>("parking_three_phase.post_align_keep_distance_m", 0.55);
  parking_three_phase_param_.post_align_enter_yaw_threshold =
    declare_parameter<double>("parking_three_phase.post_align_enter_yaw_threshold_rad", 0.10);
  parking_three_phase_param_.post_align_exit_yaw_threshold =
    declare_parameter<double>("parking_three_phase.post_align_exit_yaw_threshold_rad", 0.05);
  parking_three_phase_param_.align_kp =
    declare_parameter<double>("parking_three_phase.align_kp", 1.2);
  parking_three_phase_param_.align_min_omega =
    declare_parameter<double>("parking_three_phase.align_min_omega_rps", 0.05);
  parking_three_phase_param_.align_max_omega =
    declare_parameter<double>("parking_three_phase.align_max_omega_rps", 0.35);
  parking_three_phase_param_.hold_unlock_goal_distance =
    declare_parameter<double>("parking_three_phase.hold_unlock_goal_distance_m", 0.20);
  parking_three_phase_param_.hold_unlock_goal_yaw =
    declare_parameter<double>("parking_three_phase.hold_unlock_goal_yaw_rad", 0.20);

  diag_updater_->setHardwareID("trajectory_follower_node");

  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter<std::string>("lateral_controller_mode"));
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller_ =
        std::make_shared<mpc_lateral_controller::MpcLateralController>(*this, diag_updater_);
      break;
    }
    case LateralControllerMode::PURE_PURSUIT: {
      lateral_controller_ =
        std::make_shared<autoware::pure_pursuit::PurePursuitLateralController>(*this);
      break;
    }
    default:
      throw std::domain_error("[LateralController] invalid algorithm");
  }

  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter<std::string>("longitudinal_controller_mode"));
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller_ =
        std::make_shared<pid_longitudinal_controller::PidLongitudinalController>(
          *this, diag_updater_);
      break;
    }
    default:
      throw std::domain_error("[LongitudinalController] invalid algorithm");
  }

  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  pub_processing_time_lat_ms_ =
    create_publisher<Float64Stamped>("~/lateral/debug/processing_time_ms", 1);
  pub_processing_time_lon_ms_ =
    create_publisher<Float64Stamped>("~/longitudinal/debug/processing_time_ms", 1);
  debug_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

  if (enable_control_cmd_horizon_pub_) {
    control_cmd_horizon_pub_ = create_publisher<autoware_control_msgs::msg::ControlHorizon>(
      "~/debug/control_cmd_horizon", 1);
  }

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc") return LateralControllerMode::MPC;
  if (controller_mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;

  return LateralControllerMode::INVALID;
}

Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;

  return LongitudinalControllerMode::INVALID;
}

bool Controller::processData(rclcpp::Clock & clock)
{
  bool is_ready = true;

  const auto & logData = [&clock, this](const std::string & data_type) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval, "Waiting for %s data", data_type.c_str());
  };

  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.take_data();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(current_accel_ptr_, sub_accel_, "acceleration");
  is_ready &= getData(current_steering_ptr_, sub_steering_, "steering");
  is_ready &= getData(current_trajectory_ptr_, sub_ref_path_, "trajectory");
  is_ready &= getData(current_odometry_ptr_, sub_odometry_, "odometry");
  is_ready &= getData(current_operation_mode_ptr_, sub_operation_mode_, "operation mode");

  return is_ready;
}

bool Controller::isTimeOut(
  const trajectory_follower::LongitudinalOutput & lon_out,
  const trajectory_follower::LateralOutput & lat_out)
{
  const auto now = this->now();
  if ((now - lat_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - lon_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
}

boost::optional<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock & clock)
{
  if (!processData(clock)) {
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;
  input_data.current_operation_mode = *current_operation_mode_ptr_;

  return input_data;
}

void Controller::callbackTimerControl()
{
  autoware_control_msgs::msg::Control out;
  out.stamp = this->now();

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
    return;
  }

  // Run parking three-phase override first. If state machine takes ownership
  // (PRE_ALIGN / POST_ALIGN / HOLD), skip lateral/longitudinal controllers entirely.
  applyParkingThreePhaseOverride(*input_data, out);
  if (parking_three_phase_param_.enable && parking_three_phase_state_ != ParkingThreePhaseState::TRACK) {
    out.lateral.stamp = out.stamp;
    out.longitudinal.stamp = out.stamp;
    control_cmd_pub_->publish(out);
    published_time_publisher_->publish_if_subscribed(control_cmd_pub_, out.stamp);
    return;
  }

  // 2. check if controllers are ready
  const bool is_lat_ready = lateral_controller_->isReady(*input_data);
  const bool is_lon_ready = longitudinal_controller_->isReady(*input_data);
  if (!is_lat_ready || !is_lon_ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Control is skipped since lateral and/or longitudinal controllers are not ready to run.");
    return;
  }

  // 3. run controllers
  stop_watch_.tic("lateral");
  const auto lat_out = lateral_controller_->run(*input_data);
  publishProcessingTime(stop_watch_.toc("lateral"), pub_processing_time_lat_ms_);

  stop_watch_.tic("longitudinal");
  const auto lon_out = longitudinal_controller_->run(*input_data);
  publishProcessingTime(stop_watch_.toc("longitudinal"), pub_processing_time_lon_ms_);

  // 4. sync with each other controllers
  longitudinal_controller_->sync(lat_out.sync_data);
  lateral_controller_->sync(lon_out.sync_data);

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if (isTimeOut(lon_out, lat_out)) return;

  // 5. publish control command
  out.lateral = lat_out.control_cmd;
  out.longitudinal = lon_out.control_cmd;
  control_cmd_pub_->publish(out);

  // 6. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, out.stamp);
  publishDebugMarker(*input_data, lat_out);

  // 7. publish experimental topic
  if (enable_control_cmd_horizon_pub_) {
    const auto control_horizon =
      mergeLatLonHorizon(lat_out.control_cmd_horizon, lon_out.control_cmd_horizon, this->now());
    if (control_horizon.has_value()) {
      control_cmd_horizon_pub_->publish(control_horizon.value());
    }
  }
}

void Controller::publishDebugMarker(
  const trajectory_follower::InputData & input_data,
  const trajectory_follower::LateralOutput & lat_out) const
{
  visualization_msgs::msg::MarkerArray debug_marker_array{};

  // steer converged marker
  {
    auto marker = autoware_utils::create_default_marker(
      "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
      autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.99));
    marker.pose = input_data.current_odometry.pose.pose;

    std::stringstream ss;
    const double current = input_data.current_steering.steering_tire_angle;
    const double cmd = lat_out.control_cmd.steering_tire_angle;
    const double diff = current - cmd;
    ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
       << (lat_out.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
    marker.text = ss.str();

    debug_marker_array.markers.push_back(marker);
  }

  debug_marker_pub_->publish(debug_marker_array);
}

void Controller::publishProcessingTime(
  const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = t_ms;
  pub->publish(msg);
}

double Controller::normalizeRadian(const double rad)
{
  return std::atan2(std::sin(rad), std::cos(rad));
}

bool Controller::isSameGoal(
  const double goal_x_1, const double goal_y_1, const double goal_yaw_1, const double goal_x_2,
  const double goal_y_2, const double goal_yaw_2, const double dist_thr, const double yaw_thr)
{
  const double goal_dist = std::hypot(goal_x_1 - goal_x_2, goal_y_1 - goal_y_2);
  const double goal_yaw_diff = std::abs(normalizeRadian(goal_yaw_1 - goal_yaw_2));
  return goal_dist <= dist_thr && goal_yaw_diff <= yaw_thr;
}

bool Controller::getGoalPose(
  const trajectory_follower::InputData & input_data, double & goal_x, double & goal_y,
  double & goal_yaw) const
{
  if (input_data.current_trajectory.points.empty()) {
    return false;
  }
  const auto & goal_pose = input_data.current_trajectory.points.back().pose;
  goal_x = goal_pose.position.x;
  goal_y = goal_pose.position.y;
  goal_yaw = tf2::getYaw(goal_pose.orientation);
  return true;
}

bool Controller::getPreAlignYaw(
  const trajectory_follower::InputData & input_data, double & pre_align_yaw, bool & is_reverse) const
{
  const auto & points = input_data.current_trajectory.points;
  if (points.empty()) {
    return false;
  }

  const auto & current_pose = input_data.current_odometry.pose.pose;
  size_t target_idx = points.size() - 1;
  bool found = false;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points.at(i).pose.position;
    const double d = std::hypot(p.x - current_pose.position.x, p.y - current_pose.position.y);
    if (d >= std::max(0.0, parking_three_phase_param_.pre_align_min_target_distance)) {
      target_idx = i;
      found = true;
      break;
    }
  }
  if (!found && points.size() < 2) {
    return false;
  }
  if (!found) {
    target_idx = 1;
  }

  const auto & target = points.at(target_idx);
  const double dx = target.pose.position.x - current_pose.position.x;
  const double dy = target.pose.position.y - current_pose.position.y;
  if (std::hypot(dx, dy) < 1e-3) {
    return false;
  }

  const double current_yaw = tf2::getYaw(current_pose.orientation);
  const double head_yaw = std::atan2(dy, dx);
  constexpr double kPi = 3.14159265358979323846;
  const double tail_yaw = normalizeRadian(head_yaw + kPi);
  const double head_err = std::abs(normalizeRadian(head_yaw - current_yaw));
  const double tail_err = std::abs(normalizeRadian(tail_yaw - current_yaw));

  // Choose the smaller rotation (head or tail) to avoid two-step re-alignment.
  if (tail_err < head_err) {
    pre_align_yaw = tail_yaw;
    is_reverse = true;
  } else {
    pre_align_yaw = head_yaw;
    is_reverse = false;
  }
  return true;
}

void Controller::resetAlignLock()
{
  align_target_locked_ = false;
  align_direction_ = 0;
}

void Controller::lockAlignTarget(const double target_yaw, const double current_yaw)
{
  align_target_locked_ = true;
  align_target_yaw_ = normalizeRadian(target_yaw);
  const double yaw_error = normalizeRadian(align_target_yaw_ - current_yaw);
  align_direction_ = (yaw_error >= 0.0) ? 1 : -1;
}

double Controller::calcAlignedOmega(const double current_yaw) const
{
  if (!align_target_locked_ || align_direction_ == 0) {
    return 0.0;
  }
  const double yaw_error = normalizeRadian(align_target_yaw_ - current_yaw);
  const double max_omega = std::max(0.0, parking_three_phase_param_.align_max_omega);
  const double min_omega = std::clamp(parking_three_phase_param_.align_min_omega, 0.0, max_omega);
  const double target =
    std::clamp(parking_three_phase_param_.align_kp * std::abs(yaw_error), 0.0, max_omega);
  const double omega_abs =
    (target > 0.0) ? std::max(target, min_omega) : 0.0;
  return static_cast<double>(align_direction_) * omega_abs;
}

void Controller::setZeroLongitudinal(autoware_control_msgs::msg::Longitudinal & longitudinal) const
{
  longitudinal.velocity = 0.0F;
  longitudinal.acceleration = 0.0F;
  longitudinal.jerk = 0.0F;
  longitudinal.is_defined_acceleration = true;
  longitudinal.is_defined_jerk = true;
}

void Controller::applyParkingThreePhaseOverride(
  const trajectory_follower::InputData & input_data, autoware_control_msgs::msg::Control & out)
{
  if (!parking_three_phase_param_.enable) {
    return;
  }
  if (input_data.current_trajectory.points.empty()) {
    parking_three_phase_state_ = ParkingThreePhaseState::TRACK;
    resetAlignLock();
    hold_goal_valid_ = false;
    return;
  }

  const auto & current_pose = input_data.current_odometry.pose.pose;
  const double current_yaw = tf2::getYaw(current_pose.orientation);
  const double ego_speed = std::abs(input_data.current_odometry.twist.twist.linear.x);
  const bool is_stopped =
    ego_speed <= std::max(0.0, parking_three_phase_param_.stop_velocity_threshold);

  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_yaw = 0.0;
  const bool has_goal_pose = getGoalPose(input_data, goal_x, goal_y, goal_yaw);
  const double goal_distance =
    has_goal_pose ? std::hypot(goal_x - current_pose.position.x, goal_y - current_pose.position.y) : 0.0;
  const double goal_yaw_error =
    has_goal_pose ? normalizeRadian(goal_yaw - current_yaw) : 0.0;

  if (
    pre_align_done_goal_valid_ && has_goal_pose &&
    !isSameGoal(
      pre_align_done_goal_x_, pre_align_done_goal_y_, pre_align_done_goal_yaw_, goal_x, goal_y,
      goal_yaw, std::max(0.0, parking_three_phase_param_.hold_unlock_goal_distance),
      std::max(0.0, parking_three_phase_param_.hold_unlock_goal_yaw)))
  {
    pre_align_done_goal_valid_ = false;
  }

  double pre_align_yaw = 0.0;
  bool pre_align_reverse = false;
  const bool has_pre_align_target = getPreAlignYaw(input_data, pre_align_yaw, pre_align_reverse);
  const double pre_align_yaw_error =
    has_pre_align_target ? normalizeRadian(pre_align_yaw - current_yaw) : 0.0;
  (void)pre_align_reverse;

  if (
    parking_three_phase_state_ == ParkingThreePhaseState::HOLD && hold_goal_valid_ && has_goal_pose &&
    !isSameGoal(
      hold_goal_x_, hold_goal_y_, hold_goal_yaw_, goal_x, goal_y, goal_yaw,
      std::max(0.0, parking_three_phase_param_.hold_unlock_goal_distance),
      std::max(0.0, parking_three_phase_param_.hold_unlock_goal_yaw)))
  {
    parking_three_phase_state_ = ParkingThreePhaseState::PRE_ALIGN;
    resetAlignLock();
    hold_goal_valid_ = false;
  }

  if (parking_three_phase_state_ == ParkingThreePhaseState::TRACK) {
    const bool in_post_align_window =
      has_goal_pose &&
      goal_distance <=
        std::max(
          0.0, std::max(
                 parking_three_phase_param_.post_align_enter_distance,
                 parking_three_phase_param_.post_align_keep_distance));

    // Near goal: prioritize final-yaw alignment and do not re-enter pre-align.
    if (
      is_stopped && in_post_align_window &&
      std::abs(goal_yaw_error) >
        std::max(0.0, parking_three_phase_param_.post_align_enter_yaw_threshold))
    {
      parking_three_phase_state_ = ParkingThreePhaseState::POST_ALIGN;
      resetAlignLock();
    } else if (
      is_stopped && !in_post_align_window && has_pre_align_target && !pre_align_done_goal_valid_ &&
      std::abs(pre_align_yaw_error) >
        std::max(0.0, parking_three_phase_param_.pre_align_enter_yaw_threshold))
    {
      parking_three_phase_state_ = ParkingThreePhaseState::PRE_ALIGN;
      resetAlignLock();
    } else {
      return;
    }
  }

  if (parking_three_phase_state_ == ParkingThreePhaseState::PRE_ALIGN) {
    if (!has_pre_align_target) {
      parking_three_phase_state_ = ParkingThreePhaseState::TRACK;
      resetAlignLock();
      return;
    }
    if (!align_target_locked_) {
      lockAlignTarget(pre_align_yaw, current_yaw);
    }
    const double yaw_error = normalizeRadian(align_target_yaw_ - current_yaw);
    if (
      std::abs(yaw_error) <=
      std::max(0.0, parking_three_phase_param_.pre_align_exit_yaw_threshold))
    {
      parking_three_phase_state_ = ParkingThreePhaseState::TRACK;
      resetAlignLock();
      if (has_goal_pose) {
        pre_align_done_goal_x_ = goal_x;
        pre_align_done_goal_y_ = goal_y;
        pre_align_done_goal_yaw_ = goal_yaw;
        pre_align_done_goal_valid_ = true;
      }
      return;
    }

    setZeroLongitudinal(out.longitudinal);
    out.lateral.steering_tire_angle = static_cast<float>(calcAlignedOmega(current_yaw));
    out.lateral.steering_tire_rotation_rate = 0.0F;
    out.lateral.is_defined_steering_tire_rotation_rate = true;
    return;
  }

  if (parking_three_phase_state_ == ParkingThreePhaseState::POST_ALIGN) {
    if (!has_goal_pose) {
      parking_three_phase_state_ = ParkingThreePhaseState::TRACK;
      resetAlignLock();
      return;
    }

    if (
      goal_distance >
        std::max(
          parking_three_phase_param_.post_align_enter_distance,
          parking_three_phase_param_.post_align_keep_distance))
    {
      parking_three_phase_state_ = ParkingThreePhaseState::TRACK;
      resetAlignLock();
      return;
    }

    if (!align_target_locked_) {
      lockAlignTarget(goal_yaw, current_yaw);
    }
    const double yaw_error = normalizeRadian(align_target_yaw_ - current_yaw);
    if (
      std::abs(yaw_error) <=
      std::max(0.0, parking_three_phase_param_.post_align_exit_yaw_threshold))
    {
      parking_three_phase_state_ = ParkingThreePhaseState::HOLD;
      resetAlignLock();
      hold_goal_x_ = goal_x;
      hold_goal_y_ = goal_y;
      hold_goal_yaw_ = goal_yaw;
      hold_goal_valid_ = true;
    } else {
      setZeroLongitudinal(out.longitudinal);
      out.lateral.steering_tire_angle = static_cast<float>(calcAlignedOmega(current_yaw));
      out.lateral.steering_tire_rotation_rate = 0.0F;
      out.lateral.is_defined_steering_tire_rotation_rate = true;
      return;
    }
  }

  if (parking_three_phase_state_ == ParkingThreePhaseState::HOLD) {
    setZeroLongitudinal(out.longitudinal);
    out.lateral.steering_tire_angle = 0.0F;
    out.lateral.steering_tire_rotation_rate = 0.0F;
    out.lateral.is_defined_steering_tire_rotation_rate = true;
    return;
  }
}

std::optional<ControlHorizon> Controller::mergeLatLonHorizon(
  const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp)
{
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  autoware_control_msgs::msg::ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  // If either of the horizons has only one control, repeat the control to match the other horizon.
  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }
  if (longitudinal_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = lateral_horizon.time_step_ms;
    const auto longitudinal = longitudinal_horizon.controls.front();
    for (const auto & lateral : lateral_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
  // calculate greatest common divisor of time steps
  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const int int_a = static_cast<int>(round(a * precision));
    const int int_b = static_cast<int>(round(b * precision));
    return static_cast<double>(std::gcd(int_a, int_b)) / precision;
  };
  const double time_step_ms =
    gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
  control_horizon.time_step_ms = time_step_ms;

  const auto lateral_controls = resampleHorizonByZeroOrderHold(
    lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);
  const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
    longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

  if (lateral_controls.size() != longitudinal_controls.size()) {
    return std::nullopt;
  }

  const size_t num_steps = lateral_controls.size();
  for (size_t i = 0; i < num_steps; ++i) {
    autoware_control_msgs::msg::Control control{};
    control.stamp = stamp;
    control.lateral = lateral_controls.at(i);
    control.longitudinal = longitudinal_controls.at(i);
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_node::Controller)
