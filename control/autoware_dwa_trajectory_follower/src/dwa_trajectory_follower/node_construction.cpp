#include "autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

namespace autoware::dwa_trajectory_follower
{

DwaTrajectoryFollowerNode::DwaTrajectoryFollowerNode() : Node("dwa_trajectory_follower")
{
  // ======================================================================
  // 1) 声明参数（declare_parameter）
  // ======================================================================
  // 说明：
  // - 这里声明的参数就是该节点“可外部配置”的全集；
  // - 外部 YAML 只要与这些名字一致，就会覆盖这里的默认值；
  // - 默认值用于未配置时兜底，也作为文档化参考。

  // ---------- 基础行为参数 ----------
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 30.0);
  goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.2);
  terminal_capture_distance_m_ = declare_parameter<double>("terminal_capture_distance_m", 0.8);
  near_goal_disable_reverse_distance_m_ =
    declare_parameter<double>("near_goal_disable_reverse_distance_m", 0.8);
  near_goal_spin_stop_speed_threshold_mps_ =
    declare_parameter<double>("near_goal_spin_stop_speed_threshold_mps", 0.05);
  yaw_align_start_speed_threshold_mps_ =
    declare_parameter<double>("yaw_align_start_speed_threshold_mps", 0.03);
  yaw_align_settle_time_s_ = declare_parameter<double>("yaw_align_settle_time_s", 0.30);
  use_goal_yaw_alignment_ = declare_parameter<bool>("use_goal_yaw_alignment", true);
  goal_yaw_tolerance_rad_ = declare_parameter<double>("goal_yaw_tolerance_rad", 0.05);
  goal_yaw_align_position_hysteresis_m_ =
    declare_parameter<double>("goal_yaw_align_position_hysteresis_m", 0.3);
  goal_yaw_align_kp_ = declare_parameter<double>("goal_yaw_align_kp", 1.0);
  goal_yaw_align_max_w_rps_ = declare_parameter<double>("goal_yaw_align_max_w_rps", 0.35);
  yaw_align_max_w_accel_rps2_ = declare_parameter<double>("yaw_align_max_w_accel_rps2", 1.5);
  yaw_align_min_w_rps_ = declare_parameter<double>("yaw_align_min_w_rps", 0.05);
  yaw_align_pi_band_rad_ = declare_parameter<double>("yaw_align_pi_band_rad", 0.15);
  use_direct_goal_mode_ = declare_parameter<bool>("use_direct_goal_mode", true);

  // ---------- 差速语义输出 ----------
  // 本包固定采用差速语义：DWA求得的角速度 w 直接作为 steering_tire_angle 输出。

  // ---------- 速度/加速度约束 ----------
  min_speed_mps_ = declare_parameter<double>("min_speed_mps", 0.0);
  max_speed_mps_ = declare_parameter<double>("max_speed_mps", 2.2);
  max_accel_mps2_ = declare_parameter<double>("max_accel_mps2", 1.5);
  max_decel_mps2_ = declare_parameter<double>("max_decel_mps2", -3.0);

  // ---------- 前进/倒车切换滞回 ----------
  reverse_enter_x_threshold_m_ = declare_parameter<double>("reverse_enter_x_threshold_m", -0.30);
  reverse_exit_x_threshold_m_ = declare_parameter<double>("reverse_exit_x_threshold_m", 0.50);
  accel_kp_ = declare_parameter<double>("accel_kp", 1.2);

  // ---------- DWA 采样窗口与分辨率 ----------
  dwa_predict_time_s_ = declare_parameter<double>("dwa_predict_time_s", 2.0);
  dwa_dt_s_ = declare_parameter<double>("dwa_dt_s", 0.1);
  dwa_v_resolution_mps_ = declare_parameter<double>("dwa_v_resolution_mps", 0.10);
  dwa_w_resolution_rps_ = declare_parameter<double>("dwa_w_resolution_rps", 0.10);
  dwa_max_yaw_rate_rps_ = declare_parameter<double>("dwa_max_yaw_rate_rps", 1.0);
  dwa_max_yaw_accel_rps2_ = declare_parameter<double>("dwa_max_yaw_accel_rps2", 1.2);
  dwa_max_accel_mps2_ = declare_parameter<double>("dwa_max_accel_mps2", 1.0);

  // ---------- 经典代价权重 ----------
  w_goal_dist_ = declare_parameter<double>("w_goal_dist", 1.0);
  w_goal_heading_ = declare_parameter<double>("w_goal_heading", 0.4);
  w_speed_ = declare_parameter<double>("w_speed", 0.6);
  w_obstacle_ = declare_parameter<double>("w_obstacle", 1.4);
  w_progress_ = declare_parameter<double>("w_progress", 2.0);
  w_omega_ = declare_parameter<double>("w_omega", 0.4);

  // ---------- 推进有效性约束 ----------
  min_progress_ratio_ = declare_parameter<double>("min_progress_ratio", 0.15);

  // ---------- 归一化与障碍代价模型 ----------
  norm_eps_ = declare_parameter<double>("norm_eps", 1e-6);
  obstacle_exp_k_ = declare_parameter<double>("obstacle_exp_k", 1.2);

  // ---------- 日志参数 ----------

  // ---------- 障碍物处理参数 ----------
  obstacle_inflation_radius_m_ = declare_parameter<double>("obstacle_inflation_radius_m", 0.6);
  robot_collision_radius_m_ = declare_parameter<double>("robot_collision_radius_m", 1.0);
  obstacle_consider_range_m_ = declare_parameter<double>("obstacle_consider_range_m", 25.0);
  braking_distance_margin_m_ = declare_parameter<double>("braking_distance_margin_m", 0.2);
  braking_reaction_time_s_ = declare_parameter<double>("braking_reaction_time_s", 0.20);
  ttc_horizon_s_ = declare_parameter<double>("ttc_horizon_s", 3.0);
  ttc_cost_gain_ = declare_parameter<double>("ttc_cost_gain", 1.0);

  // ---------- 目标速度与收敛参数 ----------
  cruise_speed_mps_ = declare_parameter<double>("cruise_speed_mps", 1.0);
  near_goal_slowdown_distance_m_ = declare_parameter<double>("near_goal_slowdown_distance_m", 3.0);
  min_progress_speed_mps_ = declare_parameter<double>("min_progress_speed_mps", 0.25);
  velocity_window_time_s_ = declare_parameter<double>("velocity_window_time_s", 0.4);
  velocity_estimator_alpha_ = declare_parameter<double>("velocity_estimator_alpha", 0.35);

  enable_short_goal_pivot_after_stop_ =
    declare_parameter<bool>("enable_short_goal_pivot_after_stop", true);
  short_goal_pivot_trigger_distance_m_ =
    declare_parameter<double>("short_goal_pivot_trigger_distance_m", 5.0);
  short_goal_pivot_yaw_tolerance_rad_ =
    declare_parameter<double>("short_goal_pivot_yaw_tolerance_rad", 0.05);
  short_goal_pivot_start_speed_threshold_mps_ =
    declare_parameter<double>("short_goal_pivot_start_speed_threshold_mps", 0.05);
  short_goal_pivot_kp_ = declare_parameter<double>("short_goal_pivot_kp", 1.2);
  short_goal_pivot_max_w_rps_ = declare_parameter<double>("short_goal_pivot_max_w_rps", 0.35);
  short_goal_pivot_min_w_rps_ = declare_parameter<double>("short_goal_pivot_min_w_rps", 0.08);

  // ======================================================================
  // 2) 创建订阅器
  // ======================================================================
  // 订阅轨迹：当 use_direct_goal_mode_=false 时用于轨迹跟踪目标选点。
  sub_traj_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
    std::bind(&DwaTrajectoryFollowerNode::on_trajectory, this, std::placeholders::_1));

  // 订阅目标：当 use_direct_goal_mode_=true 时直接追踪 mission goal。
  sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS{10},
    std::bind(&DwaTrajectoryFollowerNode::on_goal, this, std::placeholders::_1));

  // 订阅里程计：提供当前位姿、速度状态。
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{10},
    std::bind(&DwaTrajectoryFollowerNode::on_odometry, this, std::placeholders::_1));

  // 订阅融合障碍（优先使用）。
  sub_predicted_objects_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS{10},
    std::bind(&DwaTrajectoryFollowerNode::on_predicted_objects, this, std::placeholders::_1));

  // 订阅检测障碍（融合障碍不可用时的回退源）。
  sub_detected_objects_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "/perception/object_recognition/detection/objects", rclcpp::QoS{10},
    std::bind(&DwaTrajectoryFollowerNode::on_detected_objects, this, std::placeholders::_1));

  // ======================================================================
  // 3) 创建发布器
  // ======================================================================
  pub_control_ = create_publisher<autoware_control_msgs::msg::Control>(
    "/control/trajectory_follower/control_cmd", 10);
  pub_gear_ =
    create_publisher<autoware_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 10);
  pub_engage_ = create_publisher<autoware_vehicle_msgs::msg::Engage>("/vehicle/engage", 10);
  pub_goal_distance_m_ = create_publisher<std_msgs::msg::Float64>(
    "/control/trajectory_follower/debug/goal_distance_m", 10);
  pub_goal_yaw_error_rad_ = create_publisher<std_msgs::msg::Float64>(
    "/control/trajectory_follower/debug/goal_yaw_error_rad", 10);

  // 运行时参数热更新回调。
  on_set_param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&DwaTrajectoryFollowerNode::on_set_parameters, this, std::placeholders::_1));

  // ======================================================================
  // 4) 创建定时器（主循环入口）
  // ======================================================================
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DwaTrajectoryFollowerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "DWA trajectory follower (C++) started.");
}

rcl_interfaces::msg::SetParametersResult DwaTrajectoryFollowerNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "control_rate_hz") control_rate_hz_ = p.as_double();
    else if (n == "goal_tolerance_m") goal_tolerance_m_ = p.as_double();
    else if (n == "terminal_capture_distance_m") terminal_capture_distance_m_ = p.as_double();
    else if (n == "near_goal_disable_reverse_distance_m") {
      near_goal_disable_reverse_distance_m_ = p.as_double();
    } else if (n == "near_goal_spin_stop_speed_threshold_mps") {
      near_goal_spin_stop_speed_threshold_mps_ = p.as_double();
    } else if (n == "yaw_align_start_speed_threshold_mps") {
      yaw_align_start_speed_threshold_mps_ = p.as_double();
    } else if (n == "yaw_align_settle_time_s") yaw_align_settle_time_s_ = p.as_double();
    else if (n == "use_goal_yaw_alignment") use_goal_yaw_alignment_ = p.as_bool();
    else if (n == "goal_yaw_tolerance_rad") goal_yaw_tolerance_rad_ = p.as_double();
    else if (n == "goal_yaw_align_position_hysteresis_m") {
      goal_yaw_align_position_hysteresis_m_ = p.as_double();
    } else if (n == "goal_yaw_align_kp") goal_yaw_align_kp_ = p.as_double();
    else if (n == "goal_yaw_align_max_w_rps") goal_yaw_align_max_w_rps_ = p.as_double();
    else if (n == "yaw_align_max_w_accel_rps2") yaw_align_max_w_accel_rps2_ = p.as_double();
    else if (n == "yaw_align_min_w_rps") yaw_align_min_w_rps_ = p.as_double();
    else if (n == "yaw_align_pi_band_rad") yaw_align_pi_band_rad_ = p.as_double();
    else if (n == "use_direct_goal_mode") use_direct_goal_mode_ = p.as_bool();
    else if (n == "cruise_speed_mps") cruise_speed_mps_ = p.as_double();
    else if (n == "min_speed_mps") min_speed_mps_ = p.as_double();
    else if (n == "max_speed_mps") max_speed_mps_ = p.as_double();
    else if (n == "min_progress_speed_mps") min_progress_speed_mps_ = p.as_double();
    else if (n == "dwa_w_resolution_rps") dwa_w_resolution_rps_ = p.as_double();
    else if (n == "dwa_predict_time_s") dwa_predict_time_s_ = p.as_double();
    else if (n == "dwa_dt_s") dwa_dt_s_ = p.as_double();
    else if (n == "dwa_v_resolution_mps") dwa_v_resolution_mps_ = p.as_double();
    else if (n == "dwa_max_yaw_accel_rps2") dwa_max_yaw_accel_rps2_ = p.as_double();
    else if (n == "dwa_max_yaw_rate_rps") dwa_max_yaw_rate_rps_ = p.as_double();
    else if (n == "dwa_max_accel_mps2") dwa_max_accel_mps2_ = p.as_double();
    else if (n == "w_goal_dist") w_goal_dist_ = p.as_double();
    else if (n == "w_goal_heading") w_goal_heading_ = p.as_double();
    else if (n == "w_speed") w_speed_ = p.as_double();
    else if (n == "w_obstacle") w_obstacle_ = p.as_double();
    else if (n == "w_progress") w_progress_ = p.as_double();
    else if (n == "w_omega") w_omega_ = p.as_double();
    else if (n == "min_progress_ratio") min_progress_ratio_ = p.as_double();
    else if (n == "norm_eps") norm_eps_ = p.as_double();
    else if (n == "obstacle_exp_k") obstacle_exp_k_ = p.as_double();
    else if (n == "obstacle_inflation_radius_m") obstacle_inflation_radius_m_ = p.as_double();
    else if (n == "robot_collision_radius_m") robot_collision_radius_m_ = p.as_double();
    else if (n == "obstacle_consider_range_m") obstacle_consider_range_m_ = p.as_double();
    else if (n == "braking_distance_margin_m") braking_distance_margin_m_ = p.as_double();
    else if (n == "braking_reaction_time_s") braking_reaction_time_s_ = p.as_double();
    else if (n == "ttc_horizon_s") ttc_horizon_s_ = p.as_double();
    else if (n == "ttc_cost_gain") ttc_cost_gain_ = p.as_double();
    else if (n == "near_goal_slowdown_distance_m") near_goal_slowdown_distance_m_ = p.as_double();
    else if (n == "velocity_window_time_s") velocity_window_time_s_ = p.as_double();
    else if (n == "velocity_estimator_alpha") velocity_estimator_alpha_ = p.as_double();
    else if (n == "enable_short_goal_pivot_after_stop") {
      enable_short_goal_pivot_after_stop_ = p.as_bool();
    } else if (n == "short_goal_pivot_trigger_distance_m") {
      short_goal_pivot_trigger_distance_m_ = p.as_double();
    } else if (n == "short_goal_pivot_yaw_tolerance_rad") {
      short_goal_pivot_yaw_tolerance_rad_ = p.as_double();
    } else if (n == "short_goal_pivot_start_speed_threshold_mps") {
      short_goal_pivot_start_speed_threshold_mps_ = p.as_double();
    } else if (n == "short_goal_pivot_kp") short_goal_pivot_kp_ = p.as_double();
    else if (n == "short_goal_pivot_max_w_rps") short_goal_pivot_max_w_rps_ = p.as_double();
    else if (n == "short_goal_pivot_min_w_rps") short_goal_pivot_min_w_rps_ = p.as_double();
    else if (n == "reverse_enter_x_threshold_m") reverse_enter_x_threshold_m_ = p.as_double();
    else if (n == "reverse_exit_x_threshold_m") reverse_exit_x_threshold_m_ = p.as_double();
    else if (n == "accel_kp") accel_kp_ = p.as_double();
    else if (n == "max_accel_mps2") max_accel_mps2_ = p.as_double();
    else if (n == "max_decel_mps2") max_decel_mps2_ = p.as_double();
  }

  return result;
}

}  // namespace autoware::dwa_trajectory_follower
