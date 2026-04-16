#include "autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp"
#include "autoware/dwa_trajectory_follower/utils.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::dwa_trajectory_follower
{

namespace
{
double point_to_oriented_box_signed_distance(
  const double px, const double py, const double ox, const double oy, const double oyaw,
  const double length, const double width)
{
  // 变换到障碍物局部坐标系（旋转矩形中心为原点）。
  const double dx = px - ox;
  const double dy = py - oy;
  const double c = std::cos(oyaw);
  const double s = std::sin(oyaw);
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;

  const double hx = 0.5 * std::max(0.05, length);
  const double hy = 0.5 * std::max(0.05, width);
  const double qx = std::abs(lx) - hx;
  const double qy = std::abs(ly) - hy;

  const double outside_x = std::max(qx, 0.0);
  const double outside_y = std::max(qy, 0.0);
  const double outside_dist = std::hypot(outside_x, outside_y);
  const double inside_dist = std::min(std::max(qx, qy), 0.0);

  // >0: 点在矩形外且为到边界距离；<0: 点在矩形内且为穿透深度。
  return outside_dist + inside_dist;
}
}  // namespace

// ============================================================================
// 回调：轨迹更新
// ============================================================================
void DwaTrajectoryFollowerNode::on_trajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  // 保存最新轨迹（后续 get_target() 会读取）。
  latest_traj_ = msg;

  // 仅更新轨迹时间戳，不再用“轨迹刷新”解除停车保持。
  // 原因：规划层会持续重发轨迹，若每次都解锁会导致到点后再次起步。
  const auto stamp = std::make_pair(msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (stamp != last_traj_stamp_) {
    last_traj_stamp_ = stamp;
  }
}

// ============================================================================
// 回调：目标点更新
// ============================================================================
void DwaTrajectoryFollowerNode::on_goal(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 停车保持状态下，只要收到新 goal 就立即解锁。
  // 不再要求“目标位置/朝向变化超过阈值”。
  if (hold_stopped_ && has_hold_goal_) {
    const double goal_shift =
      std::hypot(msg->pose.position.x - hold_goal_x_, msg->pose.position.y - hold_goal_y_);
    hold_stopped_ = false;
    reverse_mode_ = false;
    yaw_align_active_ = false;
    yaw_align_direction_ = 0;
    yaw_align_goal_locked_ = false;
    terminal_mode_active_ = false;
    // 仅在“到点停车后收到近距离新目标”时，触发一次先原地转向再起步。
    short_goal_pivot_pending_ =
      enable_short_goal_pivot_after_stop_ &&
      goal_shift <= std::max(0.0, short_goal_pivot_trigger_distance_m_);
    short_goal_pivot_reverse_latched_ = false;
    short_goal_pivot_reverse_ = false;
  } else if (hold_stopped_ && !has_hold_goal_) {
    // 理论上不会出现，但保留兜底。
    hold_stopped_ = false;
    reverse_mode_ = false;
    yaw_align_active_ = false;
    yaw_align_direction_ = 0;
    yaw_align_goal_locked_ = false;
    terminal_mode_active_ = false;
    short_goal_pivot_pending_ = false;
    short_goal_pivot_reverse_latched_ = false;
    short_goal_pivot_reverse_ = false;
  }

  // 保存最新目标。
  latest_goal_ = msg;

  // 目标时间戳只用于观测更新，不作为解锁条件。
  const auto stamp = std::make_pair(msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (stamp != last_goal_stamp_) {
    last_goal_stamp_ = stamp;
  }
}

// ============================================================================
// 回调：里程计更新
// ============================================================================
void DwaTrajectoryFollowerNode::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

// ============================================================================
// 回调：PredictedObjects -> Obstacle2D
// ============================================================================
void DwaTrajectoryFollowerNode::on_predicted_objects(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  // 每次回调先清空旧缓存，避免历史目标残留。
  predicted_obstacles_.clear();
  predicted_obstacles_.reserve(msg->objects.size());

  for (const auto & obj : msg->objects) {
    const auto & pose = obj.kinematics.initial_pose_with_covariance.pose;
    const auto & p = pose.position;
    const double yaw = yaw_from_quat(pose.orientation);
    const double length = std::max(0.1, static_cast<double>(obj.shape.dimensions.x));
    const double width = std::max(0.1, static_cast<double>(obj.shape.dimensions.y));
    predicted_obstacles_.push_back(Obstacle2D{p.x, p.y, yaw, length, width});
  }

  // 标记该障碍源已可用。
  has_predicted_obstacles_ = true;
}

// ============================================================================
// 回调：DetectedObjects -> Obstacle2D
// ============================================================================
void DwaTrajectoryFollowerNode::on_detected_objects(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  detected_obstacles_.clear();
  detected_obstacles_.reserve(msg->objects.size());

  for (const auto & obj : msg->objects) {
    // DetectedObjects 的位姿字段路径与 PredictedObjects 略有不同。
    const auto & pose = obj.kinematics.pose_with_covariance.pose;
    const auto & p = pose.position;
    const double yaw = yaw_from_quat(pose.orientation);
    const double length = std::max(0.1, static_cast<double>(obj.shape.dimensions.x));
    const double width = std::max(0.1, static_cast<double>(obj.shape.dimensions.y));
    detected_obstacles_.push_back(Obstacle2D{p.x, p.y, yaw, length, width});
  }

  has_detected_obstacles_ = true;
}

// ============================================================================
// 获取当前控制目标点
// ============================================================================
bool DwaTrajectoryFollowerNode::get_target(double & tx, double & ty) const
{
  // 模式1：直接追 mission goal。
  if (use_direct_goal_mode_) {
    if (!latest_goal_) return false;
    tx = latest_goal_->pose.position.x;
    ty = latest_goal_->pose.position.y;
    return true;
  }

  // 模式2：轨迹跟踪。
  if (!latest_traj_ || latest_traj_->points.empty()) return false;

  const auto & points = latest_traj_->points;
  const auto & odom = *latest_odom_;
  const double px = odom.pose.pose.position.x;
  const double py = odom.pose.pose.position.y;

  // 先找轨迹最近点索引。
  size_t nearest = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i].pose.position.x - px;
    const double dy = points[i].pose.position.y - py;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest = i;
    }
  }

  // 取消固定前瞻：直接跟踪最近点，避免末段集中大幅转向。
  tx = points[nearest].pose.position.x;
  ty = points[nearest].pose.position.y;
  return true;
}

// ============================================================================
// 目标点投影到车体系 x 轴
// ============================================================================
double DwaTrajectoryFollowerNode::target_x_in_vehicle_frame(
  const double px, const double py, const double yaw, const double tx, const double ty) const
{
  // map 系目标相对位移。
  const double dx = tx - px;
  const double dy = ty - py;

  // 只计算车体系前向轴投影：x>0 在前方，x<0 在后方。
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

// ============================================================================
// 前进/倒车状态机（带滞回）
// ============================================================================
bool DwaTrajectoryFollowerNode::decide_reverse_mode(const double target_x_body)
{
  if (!reverse_mode_) {
    // 当前前进：目标明显在后方才切倒车。
    if (target_x_body < reverse_enter_x_threshold_m_) {
      reverse_mode_ = true;
    }
  } else {
    // 当前倒车：目标明显回到前方才切前进。
    if (target_x_body > reverse_exit_x_threshold_m_) {
      reverse_mode_ = false;
    }
  }
  return reverse_mode_;
}

// ============================================================================
// 选择当前有效障碍源并按距离过滤
// ============================================================================
std::vector<DwaTrajectoryFollowerNode::Obstacle2D> DwaTrajectoryFollowerNode::get_active_obstacles(
  const double px, const double py) const
{
  std::vector<Obstacle2D> src;

  // 优先使用融合后的预测障碍；无则回退到检测障碍。
  if (has_predicted_obstacles_ && !predicted_obstacles_.empty()) {
    src = predicted_obstacles_;
  } else if (has_detected_obstacles_ && !detected_obstacles_.empty()) {
    src = detected_obstacles_;
  }

  // 仅保留感兴趣距离内的障碍，减少搜索负担。
  std::vector<Obstacle2D> filtered;
  filtered.reserve(src.size());
  for (const auto & o : src) {
    const double d = std::hypot(o.x - px, o.y - py);
    if (d <= obstacle_consider_range_m_) {
      filtered.push_back(o);
    }
  }
  return filtered;
}

// ============================================================================
// 单步运动学积分
// ============================================================================
DwaTrajectoryFollowerNode::SimState DwaTrajectoryFollowerNode::simulate_step(
  const SimState & s, const double v_signed, const double w, const double dt) const
{
  SimState ns = s;

  // 位置更新：以当前 yaw 为方向的欧拉积分。
  ns.x += v_signed * std::cos(s.yaw) * dt;
  ns.y += v_signed * std::sin(s.yaw) * dt;

  // 航向更新并归一化。
  ns.yaw = normalize_angle(s.yaw + w * dt);

  // 写回速度状态。
  ns.v = v_signed;
  ns.w = w;
  return ns;
}

// ============================================================================
// 评估整条候选轨迹与障碍物关系
// ============================================================================
DwaTrajectoryFollowerNode::ObstacleMetrics DwaTrajectoryFollowerNode::evaluate_obstacle_metrics(
  const std::vector<SimState> & path, const std::vector<Obstacle2D> & obstacles,
  const double dt_step) const
{
  ObstacleMetrics metrics;
  if (obstacles.empty()) {
    return metrics;
  }

  // 用离散预测点对旋转矩形障碍物计算净空（不使用障碍物运动预测）。
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & p = path[i];
    for (const auto & obs : obstacles) {
      // 机器人按圆近似，障碍物按旋转矩形；净空=点到矩形有符号距离-机器人安全半径。
      const double dist_to_box = point_to_oriented_box_signed_distance(
        p.x, p.y, obs.x, obs.y, obs.yaw, obs.length, obs.width);
      const double safety_margin = robot_collision_radius_m_ + obstacle_inflation_radius_m_;
      const double clearance = dist_to_box - safety_margin;
      metrics.min_clearance = std::min(metrics.min_clearance, clearance);

      if (clearance <= 0.0) {
        metrics.collision = true;
        // 只记录首次碰撞发生时间。
        if (!std::isfinite(metrics.ttc_s)) {
          metrics.ttc_s = (static_cast<double>(i) + 1.0) * dt_step;
        }
      }
    }
  }
  return metrics;
}

// ============================================================================
// 刹停距离估算
// ============================================================================
double DwaTrajectoryFollowerNode::calc_braking_distance(const double v_abs) const
{
  // 防止除零：给刹车减速度一个最小正值。
  const double a_brake = std::max(0.1, std::abs(max_decel_mps2_));
  const double reaction_t = std::max(0.0, braking_reaction_time_s_);

  // 公式：s = v*t_reaction + v^2/(2a)
  return std::max(0.0, v_abs) * reaction_t + (v_abs * v_abs) / (2.0 * a_brake);
}

// ============================================================================
// 组装 Control 消息
// ============================================================================
autoware_control_msgs::msg::Control DwaTrajectoryFollowerNode::make_cmd(
  const double steer, const double velocity_abs, const double acceleration) const
{
  autoware_control_msgs::msg::Control cmd;

  // 写统一时间戳，便于上下游时序一致。
  cmd.stamp = now();
  cmd.control_time = cmd.stamp;
  cmd.lateral.stamp = cmd.stamp;
  cmd.lateral.control_time = cmd.stamp;
  cmd.longitudinal.stamp = cmd.stamp;
  cmd.longitudinal.control_time = cmd.stamp;

  // 横向字段：在差速映射模式下可能写入的是 yaw rate 语义。
  cmd.lateral.steering_tire_angle = static_cast<float>(steer);
  cmd.lateral.steering_tire_rotation_rate = 0.0F;

  // 纵向字段：速度始终是正值，方向由档位单独控制。
  cmd.longitudinal.velocity = static_cast<float>(velocity_abs);
  cmd.longitudinal.acceleration = static_cast<float>(acceleration);
  cmd.longitudinal.jerk = 0.0F;

  return cmd;
}

// ============================================================================
// 发布档位
// ============================================================================
void DwaTrajectoryFollowerNode::publish_gear(const uint8_t command) const
{
  autoware_vehicle_msgs::msg::GearCommand msg;
  msg.stamp = now();
  msg.command = command;
  pub_gear_->publish(msg);
}

// ============================================================================
// 发布 engage
// ============================================================================
void DwaTrajectoryFollowerNode::publish_engage() const
{
  if (!force_engage_) return;
  autoware_vehicle_msgs::msg::Engage msg;
  msg.stamp = now();
  msg.engage = true;
  pub_engage_->publish(msg);
}

// ============================================================================
// 发布停车命令
// ============================================================================
void DwaTrajectoryFollowerNode::publish_stop()
{
  // 保持系统处于可控状态。
  publish_engage();

  // 停车时切 PARK。
  publish_gear(autoware_vehicle_msgs::msg::GearCommand::PARK);

  // 速度置 0，给一个负加速度帮助快速收敛停车。
  pub_control_->publish(make_cmd(0.0, 0.0, max_decel_mps2_));
}

// ============================================================================
// 发布行驶命令
// ============================================================================
void DwaTrajectoryFollowerNode::publish_move(
  const autoware_control_msgs::msg::Control & cmd, const bool reverse)
{
  publish_engage();

  // 根据 reverse 选择档位。
  publish_gear(
    reverse ? autoware_vehicle_msgs::msg::GearCommand::REVERSE
            : autoware_vehicle_msgs::msg::GearCommand::DRIVE);

  // DIFF_DRIVE_VEL 仿真模型按速度符号决定前后退，这里与档位语义保持一致。
  auto cmd_out = cmd;
  if (reverse) {
    cmd_out.longitudinal.velocity = -std::abs(cmd_out.longitudinal.velocity);
  } else {
    cmd_out.longitudinal.velocity = std::abs(cmd_out.longitudinal.velocity);
  }

  pub_control_->publish(cmd_out);
}

}  // namespace autoware::dwa_trajectory_follower
