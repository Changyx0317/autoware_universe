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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware/trajectory_follower_node/visibility_control.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "autoware_control_msgs/msg/longitudinal.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autoware_control_msgs/msg/detail/control_horizon__struct.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

namespace autoware::motion::control
{
using trajectory_follower::LateralHorizon;
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalHorizon;
using trajectory_follower::LongitudinalOutput;
namespace trajectory_follower_node
{

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::ControlHorizon;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_utils::StopWatch;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public rclcpp::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);
  virtual ~Controller() {}

private:
  rclcpp::TimerBase::SharedPtr timer_control_;
  double timeout_thr_sec_;
  bool enable_control_cmd_horizon_pub_{false};
  boost::optional<LongitudinalOutput> longitudinal_output_{boost::none};

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_ =
    std::make_shared<diagnostic_updater::Updater>(
      this);  // Diagnostic updater for publishing diagnostic data.

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  // Subscribers
  autoware_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory>
    sub_ref_path_{this, "~/input/reference_trajectory"};

  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "~/input/current_odometry"};

  autoware_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::SteeringReport>
    sub_steering_{this, "~/input/current_steering"};

  autoware_utils::InterProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_accel_{this, "~/input/current_accel"};

  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_operation_mode_{
    this, "~/input/current_operation_mode"};

  // Publishers
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lat_ms_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lon_ms_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::ControlHorizon>::SharedPtr control_cmd_horizon_pub_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr current_trajectory_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_ptr_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering_ptr_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_accel_ptr_;
  OperationModeState::ConstSharedPtr current_operation_mode_ptr_;

  enum class LateralControllerMode {
    INVALID = 0,
    MPC = 1,
    PURE_PURSUIT = 2,
  };
  enum class LongitudinalControllerMode {
    INVALID = 0,
    PID = 1,
  };

  /**
   * @brief compute control command, and publish periodically
   */
  boost::optional<trajectory_follower::InputData> createInputData(rclcpp::Clock & clock);
  void callbackTimerControl();
  bool processData(rclcpp::Clock & clock);
  bool isTimeOut(const LongitudinalOutput & lon_out, const LateralOutput & lat_out);
  LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
  LongitudinalControllerMode getLongitudinalControllerMode(
    const std::string & algorithm_name) const;
  void publishDebugMarker(
    const trajectory_follower::InputData & input_data,
    const trajectory_follower::LateralOutput & lat_out) const;
  /**
   * @brief merge lateral and longitudinal horizons
   * @details If one of the commands has only one control, repeat the control to match the other
   *          horizon. If each horizon has different time intervals, resample them to match the size
   *          with the greatest common divisor.
   * @param lateral_horizon lateral horizon
   * @param longitudinal_horizon longitudinal horizon
   * @param stamp stamp
   * @return merged control horizon
   */
  static std::optional<ControlHorizon> mergeLatLonHorizon(
    const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
    const rclcpp::Time & stamp);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  void publishProcessingTime(
    const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);
  StopWatch<std::chrono::milliseconds> stop_watch_;

  enum class ParkingThreePhaseState {
    PRE_ALIGN = 0,
    TRACK = 1,
    POST_ALIGN = 2,
    HOLD = 3
  };

  struct ParkingThreePhaseParam
  {
    bool enable{false};
    double stop_velocity_threshold{0.05};
    double pre_align_enter_yaw_threshold{0.10};
    double pre_align_exit_yaw_threshold{0.05};
    double pre_align_min_target_distance{0.30};
    double post_align_enter_distance{0.35};
    double post_align_keep_distance{0.55};
    double post_align_enter_yaw_threshold{0.10};
    double post_align_exit_yaw_threshold{0.05};
    double align_kp{1.2};
    double align_min_omega{0.05};
    double align_max_omega{0.35};
    double hold_unlock_goal_distance{0.20};
    double hold_unlock_goal_yaw{0.20};
  };

  ParkingThreePhaseParam parking_three_phase_param_{};
  ParkingThreePhaseState parking_three_phase_state_{ParkingThreePhaseState::TRACK};
  bool align_target_locked_{false};
  int align_direction_{0};
  double align_target_yaw_{0.0};
  double hold_goal_x_{0.0};
  double hold_goal_y_{0.0};
  double hold_goal_yaw_{0.0};
  bool hold_goal_valid_{false};
  double pre_align_done_goal_x_{0.0};
  double pre_align_done_goal_y_{0.0};
  double pre_align_done_goal_yaw_{0.0};
  bool pre_align_done_goal_valid_{false};

  static double normalizeRadian(const double rad);
  static bool isSameGoal(
    const double goal_x_1, const double goal_y_1, const double goal_yaw_1, const double goal_x_2,
    const double goal_y_2, const double goal_yaw_2, const double dist_thr, const double yaw_thr);
  bool getGoalPose(
    const trajectory_follower::InputData & input_data, double & goal_x, double & goal_y,
    double & goal_yaw) const;
  bool getPreAlignYaw(
    const trajectory_follower::InputData & input_data, double & pre_align_yaw,
    bool & is_reverse) const;
  void resetAlignLock();
  void lockAlignTarget(const double target_yaw, const double current_yaw);
  double calcAlignedOmega(const double current_yaw) const;
  void setZeroLongitudinal(autoware_control_msgs::msg::Longitudinal & longitudinal) const;
  void applyParkingThreePhaseOverride(
    const trajectory_follower::InputData & input_data, autoware_control_msgs::msg::Control & out);

  static constexpr double logger_throttle_interval = 5000;
};
}  // namespace trajectory_follower_node
}  // namespace autoware::motion::control

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
