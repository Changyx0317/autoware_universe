#ifndef AUTOWARE__DWA_TRAJECTORY_FOLLOWER__DWA_TRAJECTORY_FOLLOWER_NODE_HPP_
#define AUTOWARE__DWA_TRAJECTORY_FOLLOWER__DWA_TRAJECTORY_FOLLOWER_NODE_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace autoware::dwa_trajectory_follower
{

/**
 * @brief DWA 轨迹跟踪节点
 *
 * 设计目标：
 * 1) 读取轨迹/目标点、里程计、障碍物；
 * 2) 在控制周期内运行 DWA 采样搜索；
 * 3) 输出 Autoware 标准 Control + Gear + Engage 指令。
 *
 * 说明：
 * - 该类只负责“控制决策”；
 * - 车辆动力学积分使用简化的单轨（unicycle）模型；
 * - 方向切换（前进/倒车）通过档位实现，纵向速度保持正值语义。
 */
class DwaTrajectoryFollowerNode : public rclcpp::Node
{
public:
  /// 构造函数：完成参数声明、订阅/发布创建、定时器注册。
  DwaTrajectoryFollowerNode();

private:
  /**
   * @brief 2D 障碍物抽象
   *
   * 将不同感知消息统一成“中心点 + 朝向 + 长宽”形式（旋转矩形）。
   */
  struct Obstacle2D
  {
    double x{0.0};       ///< 障碍物中心 x（map 坐标系）
    double y{0.0};       ///< 障碍物中心 y（map 坐标系）
    double yaw{0.0};     ///< 障碍物朝向 yaw（map 坐标系）
    double length{1.0};  ///< 障碍物长度（x 方向）
    double width{1.0};   ///< 障碍物宽度（y 方向）
  };

  /**
   * @brief 仿真状态（候选轨迹积分用）
   */
  struct SimState
  {
    double x{0.0};   ///< 位置 x
    double y{0.0};   ///< 位置 y
    double yaw{0.0}; ///< 航向角
    double v{0.0};   ///< 有符号线速度（前进正、倒车负）
    double w{0.0};   ///< 角速度
  };

  /**
   * @brief DWA 单个候选的评估信息
   *
   * 包含：
   * - 原始代价项（raw）；
   * - 归一化代价项（norm）；
   * - 总代价 total_cost；
   * - 终点状态 final_state。
   */
  struct DwaCandidate
  {
    double v_abs{0.0};            ///< 候选速度幅值
    double w{0.0};                ///< 候选角速度
    double goal_dist_cost{0.0};   ///< 终点到目标点距离代价
    double goal_heading_cost{0.0};///< 终点朝向误差代价
    double speed_cost{0.0};       ///< 与巡航速度的偏差代价
    double obstacle_cost{0.0};    ///< 障碍物相关代价
    double progress_reward{0.0};  ///< 距离推进奖励
    double goal_dist_norm{0.0};     ///< 距离项归一化值
    double goal_heading_norm{0.0};  ///< 朝向项归一化值
    double speed_norm{0.0};         ///< 速度项归一化值
    double obstacle_norm{0.0};      ///< 障碍项归一化值
    double progress_cost_norm{0.0}; ///< 推进项归一化后的“代价形式”
    double total_cost{std::numeric_limits<double>::infinity()}; ///< 总代价（越小越优）
    bool collision{true};           ///< 候选轨迹是否碰撞
    SimState final_state{};         ///< 候选预测末状态
  };

  /**
   * @brief 障碍物评估结果
   */
  struct ObstacleMetrics
  {
    double min_clearance{std::numeric_limits<double>::infinity()}; ///< 最小净空
    double ttc_s{std::numeric_limits<double>::infinity()};         ///< 首次碰撞时间（无碰撞为 inf）
    bool collision{false};                                          ///< 是否发生碰撞
  };

  // ---------------------------- 回调函数 ----------------------------
  void on_trajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_predicted_objects(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);
  void on_detected_objects(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg);

  // ---------------------------- 工具函数 ----------------------------
  bool get_target(double & tx, double & ty) const;
  double target_x_in_vehicle_frame(
    double px, double py, double yaw, double tx, double ty) const;
  bool decide_reverse_mode(double target_x_body);
  std::vector<Obstacle2D> get_active_obstacles(double px, double py) const;

  SimState simulate_step(const SimState & s, double v_signed, double w, double dt) const;
  ObstacleMetrics evaluate_obstacle_metrics(
    const std::vector<SimState> & path, const std::vector<Obstacle2D> & obstacles,
    double dt_step) const;
  double calc_braking_distance(double v_abs) const;

  DwaCandidate run_dwa(
    const SimState & state, double tx, double ty, bool reverse,
    const std::vector<Obstacle2D> & obstacles);

  autoware_control_msgs::msg::Control make_cmd(
    double steer, double velocity_abs, double acceleration) const;
  void publish_gear(uint8_t command) const;
  void publish_engage() const;
  void publish_stop();
  void publish_move(const autoware_control_msgs::msg::Control & cmd, bool reverse);

  /// 主控制循环（由 wall timer 周期触发）
  void on_timer();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // ---------------------------- ROS 句柄 ----------------------------
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    sub_predicted_objects_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr sub_detected_objects_;

  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr pub_engage_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_distance_m_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_yaw_error_rad_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr on_set_param_cb_handle_;

  // ---------------------------- 运行时缓存 ----------------------------
  autoware_planning_msgs::msg::Trajectory::SharedPtr latest_traj_; ///< 最新轨迹
  geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;         ///< 最新目标点
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;                ///< 最新里程计

  std::vector<Obstacle2D> predicted_obstacles_; ///< 由 /objects 转换的障碍物
  std::vector<Obstacle2D> detected_obstacles_;  ///< 由 /detection/objects 转换的障碍物
  bool has_predicted_obstacles_{false};         ///< 是否收到过预测障碍
  bool has_detected_obstacles_{false};          ///< 是否收到过检测障碍

  bool hold_stopped_{false}; ///< 到点后保持停车标志
  bool reverse_mode_{false}; ///< 当前档位模式（false 前进，true 倒车）
  double last_cmd_w_{0.0};   ///< 上一周期输出的角速度（用于动态窗口中心）

  std::pair<int32_t, uint32_t> last_traj_stamp_{0, 0}; ///< 上次轨迹时间戳
  std::pair<int32_t, uint32_t> last_goal_stamp_{0, 0}; ///< 上次目标时间戳
  double hold_goal_x_{0.0};                             ///< 进入停车保持时的目标x
  double hold_goal_y_{0.0};                             ///< 进入停车保持时的目标y
  double hold_goal_yaw_{0.0};                           ///< 进入停车保持时的目标yaw
  bool has_hold_goal_{false};                           ///< 是否已有停车保持目标缓存

  // ---------------------------- 参数缓存 ----------------------------
  // 基础控制参数
  double control_rate_hz_{30.0};
  double goal_tolerance_m_{0.2};
  double terminal_capture_distance_m_{0.8};
  double near_goal_disable_reverse_distance_m_{0.8};
  double near_goal_spin_stop_speed_threshold_mps_{0.05};
  double yaw_align_start_speed_threshold_mps_{0.03};
  double yaw_align_start_yaw_rate_threshold_rps_{0.05};
  double yaw_align_settle_time_s_{0.30};
  bool use_goal_yaw_alignment_{true};
  double goal_yaw_tolerance_rad_{0.05};
  double goal_yaw_align_position_hysteresis_m_{0.3};
  double goal_yaw_align_kp_{1.5};
  double goal_yaw_align_max_w_rps_{0.6};
  double yaw_align_max_w_accel_rps2_{1.5};
  double yaw_align_min_w_rps_{0.05};
  double yaw_align_pi_band_rad_{0.15};
  bool hold_stop_on_goal_{true};
  bool use_direct_goal_mode_{true};
  bool force_engage_{true};

  // 差速语义输出：固定将最优角速度 w 写入 steering_tire_angle 字段

  // 纵向边界参数
  double min_speed_mps_{0.0};
  double max_speed_mps_{2.2};
  double max_accel_mps2_{1.5};
  double max_decel_mps2_{-3.0};

  // 倒车切换滞回阈值
  double reverse_enter_x_threshold_m_{-0.30};
  double reverse_exit_x_threshold_m_{0.50};
  double accel_kp_{1.2};

  // DWA 采样参数
  double dwa_predict_time_s_{2.0};
  double dwa_dt_s_{0.1};
  double dwa_v_resolution_mps_{0.10};
  double dwa_w_resolution_rps_{0.10};
  double dwa_max_yaw_rate_rps_{1.0};
  double dwa_max_yaw_accel_rps2_{1.2};
  double dwa_max_accel_mps2_{1.0};

  // 代价权重参数（经典归一化模式）
  double w_goal_dist_{1.0};
  double w_goal_heading_{0.4};
  double w_speed_{0.6};
  double w_obstacle_{1.4};
  double w_progress_{2.0};
  double w_omega_{0.4};

  // 推进约束参数
  double min_progress_ratio_{0.15};
  bool reject_low_progress_candidate_{true};

  // 归一化与障碍物代价模式
  bool enable_cost_normalization_{true};
  double norm_eps_{1e-6};
  std::string obstacle_cost_mode_{"exp"};
  double obstacle_exp_k_{1.2};

  // 调试日志参数
  bool log_cost_debug_{true};
  int log_cost_throttle_ms_{500};
  int log_top_k_{5};

  // 障碍物与安全相关参数
  bool use_obstacle_avoidance_{true};
  double obstacle_inflation_radius_m_{0.6};
  double robot_collision_radius_m_{1.0};
  double obstacle_consider_range_m_{25.0};
  bool enable_braking_distance_check_{true};
  double braking_distance_margin_m_{0.2};
  double braking_reaction_time_s_{0.20};
  bool use_ttc_cost_{true};
  double ttc_horizon_s_{3.0};
  double ttc_cost_gain_{1.0};

  // 速度目标与临近终点策略
  double cruise_speed_mps_{1.0};
  double near_goal_slowdown_distance_m_{3.0};
  double min_progress_speed_mps_{0.25};
  double velocity_window_time_s_{0.4};
  bool use_velocity_state_estimator_{true};
  double velocity_estimator_alpha_{0.35};
  bool enforce_output_min_progress_speed_{false};
  bool enable_short_goal_pivot_after_stop_{true};
  double short_goal_pivot_trigger_distance_m_{2.0};
  double short_goal_pivot_yaw_tolerance_rad_{0.12};
  double short_goal_pivot_start_speed_threshold_mps_{0.05};
  double short_goal_pivot_kp_{1.2};
  double short_goal_pivot_max_w_rps_{0.35};
  double short_goal_pivot_min_w_rps_{0.08};

  // 速度估计器内部状态
  double v_est_abs_{0.0};
  bool v_est_initialized_{false};
  double last_cmd_v_abs_{0.0};

  bool yaw_align_active_{false};
  int yaw_align_direction_{0};  // +1:逆时针, -1:顺时针
  bool yaw_align_goal_locked_{false};
  double yaw_align_goal_yaw_{0.0};
  int yaw_align_settle_counter_{0};
  bool terminal_mode_active_{false};
  bool short_goal_pivot_pending_{false};
  bool short_goal_pivot_reverse_{false};
  bool short_goal_pivot_reverse_latched_{false};
};

}  // namespace autoware::dwa_trajectory_follower

#endif  // AUTOWARE__DWA_TRAJECTORY_FOLLOWER__DWA_TRAJECTORY_FOLLOWER_NODE_HPP_
