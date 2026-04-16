#include "autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp"
#include "autoware/dwa_trajectory_follower/utils.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::dwa_trajectory_follower
{

// ============================================================================
// 主控制循环：每个控制周期执行一次
// ============================================================================
void DwaTrajectoryFollowerNode::on_timer()
{
  // --------------------------------------------------------------------------
  // 0) 基础可用性检查
  // --------------------------------------------------------------------------
  // 里程计是闭环控制的基础，缺失时立即停车。
  if (!latest_odom_) {
    publish_stop();
    return;
  }

  // --------------------------------------------------------------------------
  // 1) 读取当前车辆状态
  // --------------------------------------------------------------------------
  const auto & odom = *latest_odom_;
  const double px = odom.pose.pose.position.x;
  const double py = odom.pose.pose.position.y;
  const double yaw = yaw_from_quat(odom.pose.pose.orientation);
  const double v_meas = odom.twist.twist.linear.x;

  // --------------------------------------------------------------------------
  // 2) 获取目标点
  // --------------------------------------------------------------------------
  double tx = 0.0;
  double ty = 0.0;
  if (!get_target(tx, ty)) {
    // 没有可用目标，不应该继续运行控制，直接停车。
    publish_stop();
    return;
  }

  // ---------------- 终点判定目标（与DWA前瞻跟踪目标解耦） ----------------
  // 说明：
  // - DWA 的控制目标仍然使用 get_target() 给出的前瞻点 (tx, ty)；
  // - 但“到达终点并执行原地朝向对齐”必须基于真实终点（goal 或轨迹末点）判定。
  //   否则在不同代价函数下（MATLAB/经典）会出现“有的能触发到点，有的不能触发”的不一致。
  double goal_pos_x = tx;
  double goal_pos_y = ty;
  bool has_goal_yaw = false;
  double goal_yaw = yaw;
  if (latest_goal_) {
    goal_pos_x = latest_goal_->pose.position.x;
    goal_pos_y = latest_goal_->pose.position.y;
    goal_yaw = yaw_from_quat(latest_goal_->pose.orientation);
    has_goal_yaw = true;
  } else if (latest_traj_ && !latest_traj_->points.empty()) {
    const auto & last_pt = latest_traj_->points.back().pose;
    goal_pos_x = last_pt.position.x;
    goal_pos_y = last_pt.position.y;
    goal_yaw = yaw_from_quat(last_pt.orientation);
    has_goal_yaw = true;
  }

  // 当前位置到“终点”的平面距离（不是前瞻点距离）。
  const double goal_dist = std::hypot(goal_pos_x - px, goal_pos_y - py);
  const double goal_yaw_for_error = has_goal_yaw ? goal_yaw : std::atan2(ty - py, tx - px);
  const double goal_yaw_error = normalize_angle(goal_yaw_for_error - yaw);
  {
    std_msgs::msg::Float64 msg;
    msg.data = goal_dist;
    pub_goal_distance_m_->publish(msg);
  }
  {
    std_msgs::msg::Float64 msg;
    msg.data = goal_yaw_error;
    pub_goal_yaw_error_rad_->publish(msg);
  }

  // 状态触发版“近距离先原地转向再起步”：
  // 不再强依赖 on_goal 回调时机。只要满足“近距离 + 低速 + 航向未对齐 + 非终点接管”，
  // 就自动进入 short_goal_pivot_pending_。
  if (
    enable_short_goal_pivot_after_stop_ && !short_goal_pivot_pending_ && !hold_stopped_ &&
    latest_goal_) {
    const double terminal_enter_dist_local =
      std::max(goal_tolerance_m_, terminal_capture_distance_m_);
    const bool near_short_goal = goal_dist <= std::max(0.0, short_goal_pivot_trigger_distance_m_);
    const bool stopped_for_pivot =
      std::abs(v_meas) <= std::max(0.0, short_goal_pivot_start_speed_threshold_mps_);
    const bool outside_terminal_capture = goal_dist > terminal_enter_dist_local;
    if (near_short_goal && stopped_for_pivot && outside_terminal_capture) {
      const double heading_to_goal = std::atan2(goal_pos_y - py, goal_pos_x - px);
      const double yaw_err_to_goal = normalize_angle(heading_to_goal - yaw);
      const double yaw_err_tail_to_goal = normalize_angle(heading_to_goal + M_PI - yaw);
      const double yaw_err_min =
        std::min(std::abs(yaw_err_to_goal), std::abs(yaw_err_tail_to_goal));
      if (yaw_err_min > std::max(0.0, short_goal_pivot_yaw_tolerance_rad_)) {
        short_goal_pivot_pending_ = true;
        short_goal_pivot_reverse_latched_ = false;
        short_goal_pivot_reverse_ = false;
      }
    }
  }

  // 到点停车后收到“近距离新目标”时：先原地对齐目标方向，再恢复行驶。
  if (short_goal_pivot_pending_) {
    if (goal_dist > std::max(0.0, short_goal_pivot_trigger_distance_m_)) {
      short_goal_pivot_pending_ = false;
      short_goal_pivot_reverse_latched_ = false;
    } else {
      const bool stopped_for_pivot =
        std::abs(v_meas) <= std::max(0.0, short_goal_pivot_start_speed_threshold_mps_);
      if (!stopped_for_pivot) {
        last_cmd_w_ = 0.0;
        last_cmd_v_abs_ = 0.0;
        publish_stop();
        return;
      }

      const double heading_to_goal = std::atan2(goal_pos_y - py, goal_pos_x - px);
      const double yaw_err_to_goal = normalize_angle(heading_to_goal - yaw);
      const double yaw_err_tail_to_goal = normalize_angle(heading_to_goal + M_PI - yaw);
      if (!short_goal_pivot_reverse_latched_) {
        short_goal_pivot_reverse_ = std::abs(yaw_err_tail_to_goal) < std::abs(yaw_err_to_goal);
        short_goal_pivot_reverse_latched_ = true;
      }
      const double yaw_err_for_pivot = short_goal_pivot_reverse_ ? yaw_err_tail_to_goal : yaw_err_to_goal;
      const double yaw_err_abs = std::abs(yaw_err_for_pivot);

      if (yaw_err_abs <= std::max(0.0, short_goal_pivot_yaw_tolerance_rad_)) {
        short_goal_pivot_pending_ = false;
        short_goal_pivot_reverse_latched_ = false;
        reverse_mode_ = short_goal_pivot_reverse_;
      } else {
        double w_pivot = short_goal_pivot_kp_ * yaw_err_for_pivot;
        w_pivot = std::clamp(
          w_pivot, -std::max(0.0, short_goal_pivot_max_w_rps_),
          std::max(0.0, short_goal_pivot_max_w_rps_));
        if (std::abs(w_pivot) > 0.0) {
          const double w_floor = std::max(0.0, short_goal_pivot_min_w_rps_);
          w_pivot = std::copysign(std::max(std::abs(w_pivot), w_floor), w_pivot);
        }
        auto pivot_cmd = make_cmd(w_pivot, 0.0, 0.0);
        publish_move(pivot_cmd, short_goal_pivot_reverse_);
        last_cmd_w_ = w_pivot;
        last_cmd_v_abs_ = 0.0;
        return;
      }
    }
  }

  // 终点接管模式：进入接管半径后，完全绕过DWA，执行“刹停->原地对齐”。
  const double terminal_enter_dist = std::max(goal_tolerance_m_, terminal_capture_distance_m_);
  if (!terminal_mode_active_ && goal_dist <= terminal_enter_dist) {
    terminal_mode_active_ = true;
  }
  // 一旦进入终点接管，就保持接管直到本轮“停车保持/对齐完成/新目标解锁”。
  // 目的：避免接管边界附近在 DWA 与对齐逻辑之间来回切换，导致“先反向再回正”。

  // --------------------------------------------------------------------------
  // 3) 到点/保持停车逻辑
  // --------------------------------------------------------------------------
  const double effective_goal_reach_dist = goal_tolerance_m_;
  const bool in_goal_position_window = goal_dist <= effective_goal_reach_dist;
  const bool in_goal_align_hysteresis_window =
    yaw_align_active_ &&
    goal_dist <= (effective_goal_reach_dist + std::max(0.0, goal_yaw_align_position_hysteresis_m_));
  const bool should_keep_yaw_align =
    yaw_align_active_ && use_goal_yaw_alignment_ && has_goal_yaw && !hold_stopped_;

  // 一旦进入终点位置窗口，立即锁存“终点对齐阶段”。
  // 这样即使车辆尚未完全停稳，也不会掉回 DWA，避免控制权在 DWA/对齐逻辑之间反复横跳。
  if ((in_goal_position_window || terminal_mode_active_) && use_goal_yaw_alignment_ && has_goal_yaw &&
      !hold_stopped_) {
    yaw_align_active_ = true;
    if (!yaw_align_goal_locked_) {
      yaw_align_goal_yaw_ = goal_yaw;
      yaw_align_goal_locked_ = true;
    }
  }

  if (
    terminal_mode_active_ || in_goal_position_window || in_goal_align_hysteresis_window ||
    should_keep_yaw_align) {
    // 两阶段到点逻辑：
    // 1) 先到位置（本条件）
    // 2) 再原地对齐终点朝向（可选）
    if (use_goal_yaw_alignment_ && has_goal_yaw) {
      // 先停车再对齐：仅使用线速度做停稳判定，并要求连续维持一段时间。
      // 说明：履带/差速底盘在原地摩擦阶段角速度可能有噪声，若强依赖 yaw_rate
      // 容易出现“永远进不了对齐”的卡滞。
      const bool lin_ok =
        std::abs(v_meas) <= std::max(0.0, yaw_align_start_speed_threshold_mps_);
      const int settle_cycles = std::max(
        1, static_cast<int>(std::ceil(
             std::max(0.0, yaw_align_settle_time_s_) * std::max(1.0, control_rate_hz_))));

      if (!lin_ok) {
        yaw_align_settle_counter_ = 0;
        last_cmd_w_ = 0.0;
        last_cmd_v_abs_ = 0.0;
        publish_stop();
        return;
      }
      yaw_align_settle_counter_++;
      if (yaw_align_settle_counter_ < settle_cycles) {
        last_cmd_w_ = 0.0;
        last_cmd_v_abs_ = 0.0;
        publish_stop();
        return;
      }

      const double yaw_goal_ref = yaw_align_goal_locked_ ? yaw_align_goal_yaw_ : goal_yaw;
      const double yaw_err = normalize_angle(yaw_goal_ref - yaw);
      const double yaw_err_abs = std::abs(yaw_err);

      // 朝向已对齐：进入停车保持。
      if (yaw_err_abs <= goal_yaw_tolerance_rad_) {
        yaw_align_active_ = false;
        yaw_align_direction_ = 0;
        yaw_align_goal_locked_ = false;
        yaw_align_settle_counter_ = 0;
        terminal_mode_active_ = false;
        if (hold_stop_on_goal_) {
          hold_stopped_ = true;
          hold_goal_x_ = goal_pos_x;
          hold_goal_y_ = goal_pos_y;
          hold_goal_yaw_ = yaw_goal_ref;
          has_hold_goal_ = true;
        }
        publish_stop();
        return;
      }

      // 朝向未对齐：原地转向（v=0）。
      // 采用“方向锁定 + 近目标降幅”策略：
      // 1) 首次进入对齐时锁定顺/逆时针方向；
      // 2) 未进入容差前不允许方向翻转，避免“先反向再正向”的摆头。
      yaw_align_active_ = true;
      if (yaw_align_direction_ == 0) {
        // 进入对齐阶段后，方向仅由当前 yaw 误差决定并锁定。
        // 不再参考上一帧 DWA 角速度，避免继承“进站瞬间”的错误符号导致首次反向扭头。
        (void)yaw_align_pi_band_rad_;
        yaw_align_direction_ = (yaw_err >= 0.0) ? 1 : -1;
      }
      const double slowdown_start =
        std::max(goal_yaw_tolerance_rad_ * 4.0, 0.2);
      const double align_scale = std::clamp(yaw_err_abs / slowdown_start, 0.05, 1.0);
      const double w_align_target = goal_yaw_align_kp_ * yaw_err_abs * align_scale;
      const double w_align_mag =
        std::clamp(w_align_target, 0.0, goal_yaw_align_max_w_rps_);
      const double w_align_mag_with_floor =
        (w_align_mag > 0.0 && yaw_err_abs > goal_yaw_tolerance_rad_)
          ? std::max(w_align_mag, std::max(0.0, yaw_align_min_w_rps_))
          : 0.0;
      const double desired_w = static_cast<double>(yaw_align_direction_) * w_align_mag_with_floor;
      const double dt_ctrl = 1.0 / std::max(1.0, control_rate_hz_);
      const double max_dw = std::max(0.0, yaw_align_max_w_accel_rps2_) * dt_ctrl;
      double w_align = desired_w;
      if (max_dw > 0.0) {
        w_align = std::clamp(desired_w, last_cmd_w_ - max_dw, last_cmd_w_ + max_dw);
      }
      w_align = std::clamp(w_align, -goal_yaw_align_max_w_rps_, goal_yaw_align_max_w_rps_);
      auto yaw_align_cmd = make_cmd(w_align, 0.0, 0.0);
      reverse_mode_ = false;
      publish_move(yaw_align_cmd, false);
      last_cmd_w_ = w_align;
      last_cmd_v_abs_ = 0.0;
      return;
    } else if (hold_stop_on_goal_) {
      // 未启用终点朝向对齐时，按原逻辑到点即停车保持。
      yaw_align_active_ = false;
      yaw_align_direction_ = 0;
      yaw_align_goal_locked_ = false;
      yaw_align_settle_counter_ = 0;
      terminal_mode_active_ = false;
      hold_stopped_ = true;
      hold_goal_x_ = goal_pos_x;
      hold_goal_y_ = goal_pos_y;
      hold_goal_yaw_ = goal_yaw;
      has_hold_goal_ = true;
    }
    publish_stop();
    return;
  }
  if (hold_stopped_) {
    yaw_align_settle_counter_ = 0;
    publish_stop();
    return;
  }

  // --------------------------------------------------------------------------
  // 4) 前进/倒车模式判定
  // --------------------------------------------------------------------------
  const double tx_body = target_x_in_vehicle_frame(px, py, yaw, tx, ty);
  bool reverse = false;

  // 直接目标模式下，如果目标明确在车后方，优先强制倒车。
  // 这样可以避免“近目标禁用倒车”或状态滞回导致的持续前进。
  const bool force_reverse_for_direct_goal =
    use_direct_goal_mode_ && latest_goal_ &&
    (target_x_in_vehicle_frame(
       px, py, yaw, latest_goal_->pose.position.x, latest_goal_->pose.position.y) <
     reverse_enter_x_threshold_m_);
  if (force_reverse_for_direct_goal) {
    reverse_mode_ = true;
    reverse = true;
  } else if (goal_dist > near_goal_disable_reverse_distance_m_) {
    // 终点附近禁用倒车切换，避免轻微冲过头触发“追身后目标”导致转圈。
    reverse = decide_reverse_mode(tx_body);
  } else {
    reverse_mode_ = false;
  }
  yaw_align_settle_counter_ = 0;

  // --------------------------------------------------------------------------
  // 5) 速度状态估计（用于动态窗口中心）
  // --------------------------------------------------------------------------
  const double current_v_abs_meas = std::abs(v_meas);
  double current_v_abs_for_window = current_v_abs_meas;

  if (use_velocity_state_estimator_) {
    // 首次初始化。
    if (!v_est_initialized_) {
      v_est_abs_ = current_v_abs_meas;
      v_est_initialized_ = true;
    }

    // alpha 越大越信任当前测量，越小越平滑。
    const double alpha = std::clamp(velocity_estimator_alpha_, 0.0, 1.0);

    // trend 取“估计速度”和“上次命令速度”的较大值，缓解响应滞后导致的速度锁死。
    const double trend = std::max(v_est_abs_, last_cmd_v_abs_);
    v_est_abs_ = alpha * current_v_abs_meas + (1.0 - alpha) * trend;
    current_v_abs_for_window = v_est_abs_;
  }

  // --------------------------------------------------------------------------
  // 6) 组装 DWA 初始状态
  // --------------------------------------------------------------------------
  SimState s;
  s.x = px;
  s.y = py;
  s.yaw = yaw;
  // 倒车模式下写入负速度，确保状态连续性。
  s.v = reverse ? -current_v_abs_for_window : current_v_abs_for_window;
  s.w = last_cmd_w_;

  // --------------------------------------------------------------------------
  // 7) 获取当前有效障碍物
  // --------------------------------------------------------------------------
  const auto obstacles = get_active_obstacles(px, py);

  // --------------------------------------------------------------------------
  // 8) 运行 DWA 搜索
  // --------------------------------------------------------------------------
  auto best = run_dwa(s, tx, ty, reverse, obstacles);

  // 无可行解或碰撞解，进入安全停车。
  if (!std::isfinite(best.total_cost) || best.collision) {
    publish_stop();
    return;
  }

  // --------------------------------------------------------------------------
  // 9) 将 (v,w) 映射为控制消息
  // --------------------------------------------------------------------------
  // 纯差速语义：直接把最优角速度 w 写入 steering_tire_angle 字段。
  // 终点附近对角速度做“幅值缩放 + 斜率限制”，抑制左右抖振。
  double steer = best.w;

  // --------------------------------------------------------------------------
  // 10) 临近目标自动降速
  // --------------------------------------------------------------------------
  const double near_scale =
    std::clamp(goal_dist / std::max(0.3, near_goal_slowdown_distance_m_), 0.2, 1.0);
  // 与线速度同源的近终点缩放：越接近终点，允许的角速度越小。
  // 这样可避免在末端对极小位置误差做大幅左右修正。
  steer *= near_scale;

  // 角速度变化率限制：避免相邻周期在正负方向快速翻转。
  {
    const double dt_ctrl_main = 1.0 / std::max(1.0, control_rate_hz_);
    const double max_dw = std::max(0.0, dwa_max_yaw_accel_rps2_) * dt_ctrl_main;
    if (max_dw > 0.0) {
      steer = std::clamp(steer, last_cmd_w_ - max_dw, last_cmd_w_ + max_dw);
    }
  }

  const double output_min_speed = enforce_output_min_progress_speed_
                                    ? std::min(min_progress_speed_mps_, max_speed_mps_)
                                    : min_speed_mps_;

  double desired_v_abs = std::clamp(best.v_abs * near_scale, output_min_speed, max_speed_mps_);

  // 终点附近禁止“零速转向”：
  // 当已经足够接近目标且纵向速度目标很小，直接停车保持，避免原地转圈。
  if (
    goal_dist <= near_goal_disable_reverse_distance_m_ &&
    desired_v_abs <= near_goal_spin_stop_speed_threshold_mps_) {
    if (hold_stop_on_goal_) {
      hold_stopped_ = true;
      hold_goal_x_ = goal_pos_x;
      hold_goal_y_ = goal_pos_y;
      hold_goal_yaw_ = goal_yaw;
      has_hold_goal_ = true;
    }
    publish_stop();
    return;
  }

  // 用简单比例项生成加速度指令，并进行上下限裁剪。
  const double current_v_abs = current_v_abs_meas;
  double accel_cmd = (desired_v_abs - current_v_abs) * accel_kp_;
  accel_cmd = std::clamp(accel_cmd, max_decel_mps2_, max_accel_mps2_);

  // 组装并发布控制命令。
  auto cmd = make_cmd(steer, desired_v_abs, accel_cmd);
  publish_move(cmd, reverse);

  // --------------------------------------------------------------------------
  // 11) 更新缓存
  // --------------------------------------------------------------------------
  // 下一周期动态窗口会围绕本周期输出继续搜索。
  last_cmd_w_ = best.w;
  last_cmd_v_abs_ = desired_v_abs;
}

}  // namespace autoware::dwa_trajectory_follower
