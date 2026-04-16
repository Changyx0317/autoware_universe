#include "autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp"
#include "autoware/dwa_trajectory_follower/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace autoware::dwa_trajectory_follower
{

DwaTrajectoryFollowerNode::DwaCandidate DwaTrajectoryFollowerNode::run_dwa(
  const SimState & state, const double tx, const double ty, const bool reverse,
  const std::vector<Obstacle2D> & obstacles)
{
  // best 默认 total_cost=inf，表示“尚未找到可行解”。
  DwaCandidate best;

  // ========================================================================
  // 1) 构造动态窗口（Dynamic Window）
  // ========================================================================
  // 控制周期，用于角速度窗口。
  const double dt_ctrl = 1.0 / std::max(1.0, control_rate_hz_);

  // 速度窗口时域：允许比 dt_ctrl 更大，缓解上层-下层响应滞后。
  const double v_window_dt = std::max(dt_ctrl, velocity_window_time_s_);

  // 当前速度幅值（采样窗口围绕它扩展）。
  const double current_v_abs = std::abs(state.v);

  // 起点到目标距离，用于最低搜索速度策略。
  const double dist_from_start_to_goal = std::hypot(tx - state.x, ty - state.y);
  const double min_search_speed =
    dist_from_start_to_goal > (goal_tolerance_m_ * 3.0) ? min_progress_speed_mps_ : min_speed_mps_;

  // 速度采样区间。
  const double v_low = std::max(min_speed_mps_, current_v_abs - dwa_max_accel_mps2_ * v_window_dt);
  const double v_high = std::min(
    max_speed_mps_, std::max(current_v_abs + dwa_max_accel_mps2_ * v_window_dt, min_search_speed));

  // 角速度采样区间（围绕上一时刻角速度并受角加速度约束）。
  const double w_low = std::max(-dwa_max_yaw_rate_rps_, state.w - dwa_max_yaw_accel_rps2_ * v_window_dt);
  const double w_high = std::min(dwa_max_yaw_rate_rps_, state.w + dwa_max_yaw_accel_rps2_ * v_window_dt);

  // 目标巡航速度幅值（用于 speed_cost）。
  const double v_target_abs = std::clamp(cruise_speed_mps_, min_speed_mps_, max_speed_mps_);

  // 保存所有通过约束筛选的候选。
  std::vector<DwaCandidate> valid_candidates;

  // ========================================================================
  // 2) 二维采样 (v, w)
  // ========================================================================
  for (double v_abs = v_low; v_abs <= v_high + 1e-6; v_abs += std::max(0.01, dwa_v_resolution_mps_)) {
    // 纯差速语义：角速度上限固定由 dwa_max_yaw_rate_rps_ 决定。
    const double kinematic_w_limit = dwa_max_yaw_rate_rps_;

    // 角速度评估区间为“动态窗口”和“运动学上限”两者交集。
    const double w_eval_low = std::max(w_low, -kinematic_w_limit);
    const double w_eval_high = std::min(w_high, kinematic_w_limit);

    for (double w = w_eval_low; w <= w_eval_high + 1e-6; w += std::max(0.01, dwa_w_resolution_rps_)) {
      DwaCandidate cand;
      cand.v_abs = v_abs;
      cand.w = w;

      // --------------------------------------------------------------------
      // 2.1 前向预测轨迹
      // --------------------------------------------------------------------
      std::vector<SimState> path;
      path.reserve(static_cast<size_t>(std::ceil(dwa_predict_time_s_ / std::max(0.02, dwa_dt_s_))) + 2);
      path.push_back(state);

      SimState sim = state;
      const double v_signed = reverse ? -v_abs : v_abs;
      for (double t = 0.0; t < dwa_predict_time_s_; t += std::max(0.02, dwa_dt_s_)) {
        sim = simulate_step(sim, v_signed, w, std::max(0.02, dwa_dt_s_));
        path.push_back(sim);
      }
      cand.final_state = sim;

      // --------------------------------------------------------------------
      // 2.2 目标相关代价
      // --------------------------------------------------------------------
      const double dx = tx - sim.x;
      const double dy = ty - sim.y;
      const double dist_to_goal = std::hypot(dx, dy);
      cand.goal_dist_cost = dist_to_goal;

      // 推进奖励：起点到目标距离 - 终点到目标距离（越大越好）。
      cand.progress_reward = std::max(0.0, dist_from_start_to_goal - dist_to_goal);

      // 推进硬约束：推进过少直接丢弃候选。
      const double expected_progress = v_abs * dwa_predict_time_s_ * min_progress_ratio_;
      if (reject_low_progress_candidate_ && cand.progress_reward < expected_progress) {
        continue;
      }

      // 朝向误差：终点朝向与“目标方向（倒车时加 pi）”的误差。
      const double target_heading = std::atan2(dy, dx);
      const double heading_ref = reverse ? normalize_angle(target_heading + M_PI) : target_heading;
      const double heading_err = std::abs(normalize_angle(heading_ref - sim.yaw));
      cand.goal_heading_cost = heading_err;

      // 速度代价：离巡航速度越远，代价越大。
      cand.speed_cost = std::max(0.0, v_target_abs - v_abs);

      // --------------------------------------------------------------------
      // 2.3 障碍相关代价与约束
      // --------------------------------------------------------------------
      if (use_obstacle_avoidance_) {
        const double dt_step = std::max(0.02, dwa_dt_s_);
        const auto metrics = evaluate_obstacle_metrics(path, obstacles, dt_step);

        // 发生碰撞的候选直接丢弃。
        if (metrics.collision) {
          cand.collision = true;
          continue;
        }
        cand.collision = false;

        // 刹停距离硬约束：净空小于“可刹停距离 + 安全余量”则丢弃。
        if (enable_braking_distance_check_) {
          const double required_clearance =
            calc_braking_distance(v_abs) + std::max(0.0, braking_distance_margin_m_);
          if (metrics.min_clearance < required_clearance) {
            continue;
          }
        }

        // 障碍代价两种模式：反比模式 or 指数模式。
        if (obstacle_cost_mode_ == "inv") {
          cand.obstacle_cost = 1.0 / (norm_eps_ + metrics.min_clearance);
        } else {
          cand.obstacle_cost = std::exp(-obstacle_exp_k_ * metrics.min_clearance);
        }

        // TTC 代价附加项：越接近碰撞，代价越高。
        if (use_ttc_cost_ && std::isfinite(metrics.ttc_s)) {
          const double horizon = std::max(0.1, ttc_horizon_s_);
          const double ttc_norm =
            std::clamp((horizon - std::min(metrics.ttc_s, horizon)) / horizon, 0.0, 1.0);
          cand.obstacle_cost += std::max(0.0, ttc_cost_gain_) * ttc_norm;
        }

      } else {
        // 不避障模式下，障碍代价固定为 0。
        cand.collision = false;
        cand.obstacle_cost = 0.0;
      }

      // 到这里说明候选可行，加入集合。
      valid_candidates.push_back(cand);
    }
  }

  // 若无可行候选，返回 inf 解让上层触发停车。
  if (valid_candidates.empty()) {
    return best;
  }

  // ========================================================================
  // 3) 统计各代价值域（用于归一化）
  // ========================================================================
  auto minmax_raw = [](const std::vector<DwaCandidate> & cands, auto getter) {
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto & c : cands) {
      const double v = getter(c);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
    }
    return std::pair<double, double>{min_v, max_v};
  };

  const auto mm_dist = minmax_raw(valid_candidates, [](const auto & c) { return c.goal_dist_cost; });
  const auto mm_heading = minmax_raw(valid_candidates, [](const auto & c) { return c.goal_heading_cost; });
  const auto mm_speed = minmax_raw(valid_candidates, [](const auto & c) { return c.speed_cost; });
  const auto mm_obs = minmax_raw(valid_candidates, [](const auto & c) { return c.obstacle_cost; });
  const auto mm_prog = minmax_raw(valid_candidates, [](const auto & c) { return c.progress_reward; });

  // ========================================================================
  // 4) 计算总分（经典模式：min-max 归一化后线性加权求和）
  // ========================================================================
  auto norm = [this](const double x, const double min_v, const double max_v) {
    const double range = max_v - min_v;
    if (range <= norm_eps_) return 0.0;
    return std::clamp((x - min_v) / (range + norm_eps_), 0.0, 1.0);
  };

  for (auto & c : valid_candidates) {
    if (enable_cost_normalization_) {
      c.goal_dist_norm = norm(c.goal_dist_cost, mm_dist.first, mm_dist.second);
      c.goal_heading_norm = norm(c.goal_heading_cost, mm_heading.first, mm_heading.second);
      c.speed_norm = norm(c.speed_cost, mm_speed.first, mm_speed.second);
      c.obstacle_norm = norm(c.obstacle_cost, mm_obs.first, mm_obs.second);
      const double progress_norm = norm(c.progress_reward, mm_prog.first, mm_prog.second);
      // 奖励转代价：推进越大，代价越小。
      c.progress_cost_norm = 1.0 - progress_norm;
    } else {
      // 调试时可关闭归一化，直接使用原始值。
      c.goal_dist_norm = c.goal_dist_cost;
      c.goal_heading_norm = c.goal_heading_cost;
      c.speed_norm = c.speed_cost;
      c.obstacle_norm = c.obstacle_cost;
      c.progress_cost_norm = -c.progress_reward;
    }

    // 角速度惩罚：抑制过大角速度造成的“原地打转”倾向。
    const double omega_cost = std::abs(c.w) / std::max(0.1, dwa_max_yaw_rate_rps_);

    c.total_cost = w_goal_dist_ * c.goal_dist_norm +
                   w_goal_heading_ * c.goal_heading_norm +
                   w_speed_ * c.speed_norm +
                   w_obstacle_ * c.obstacle_norm +
                   w_progress_ * c.progress_cost_norm +
                   w_omega_ * omega_cost;
  }

  // ========================================================================
  // 5) 选最优候选
  // ========================================================================
  std::sort(
    valid_candidates.begin(), valid_candidates.end(),
    [](const DwaCandidate & a, const DwaCandidate & b) { return a.total_cost < b.total_cost; });
  best = valid_candidates.front();

  // ========================================================================
  // 6) 调试日志输出
  // ========================================================================
  if (log_cost_debug_) {
    // 输出最优候选的 raw/norm/total，便于调参定位。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_cost_throttle_ms_,
      "DWA BEST v=%.3f w=%.3f raw[d=%.3f h=%.3f s=%.3f o=%.3f p=%.3f] norm[d=%.3f h=%.3f s=%.3f o=%.3f pc=%.3f] total=%.3f",
      best.v_abs, best.w, best.goal_dist_cost, best.goal_heading_cost, best.speed_cost,
      best.obstacle_cost, best.progress_reward, best.goal_dist_norm, best.goal_heading_norm,
      best.speed_norm, best.obstacle_norm, best.progress_cost_norm, best.total_cost);

    // 输出各项取值范围，便于判断归一化是否“压扁”。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_cost_throttle_ms_,
      "DWA RANGE dist[%.3f,%.3f] heading[%.3f,%.3f] speed[%.3f,%.3f] obs[%.3f,%.3f] progress[%.3f,%.3f] valid=%zu",
      mm_dist.first, mm_dist.second, mm_heading.first, mm_heading.second, mm_speed.first,
      mm_speed.second, mm_obs.first, mm_obs.second, mm_prog.first, mm_prog.second,
      valid_candidates.size());

    // 输出 top-k 候选，观察“最优为何胜出”。
    if (log_top_k_ > 0) {
      std::ostringstream oss;
      oss << "DWA TOPK";
      const size_t k = std::min(static_cast<size_t>(log_top_k_), valid_candidates.size());
      for (size_t i = 0; i < k; ++i) {
        const auto & c = valid_candidates[i];
        oss << " |#" << i
            << " v=" << std::fixed << std::setprecision(2) << c.v_abs
            << " w=" << c.w
            << " nd=" << c.goal_dist_norm
            << " nh=" << c.goal_heading_norm
            << " ns=" << c.speed_norm
            << " no=" << c.obstacle_norm
            << " np=" << c.progress_cost_norm
            << " J=" << c.total_cost;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_cost_throttle_ms_, "%s", oss.str().c_str());
    }
  }

  return best;
}

}  // namespace autoware::dwa_trajectory_follower
