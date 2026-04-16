#ifndef AUTOWARE__DWA_TRAJECTORY_FOLLOWER__UTILS_HPP_
#define AUTOWARE__DWA_TRAJECTORY_FOLLOWER__UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace autoware::dwa_trajectory_follower
{

/**
 * @brief 四元数转偏航角（yaw）
 *
 * 说明：
 * - 控制平面为 2D（x-y），这里只取绕 z 轴的角度。
 * - 返回范围由 atan2 决定，天然在 [-pi, pi]。
 */
inline double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  // sin(yaw) 与 cos(yaw) 的组合形式，避免直接欧拉角分解引入的中间误差。
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief 角度归一化到 [-pi, pi]
 *
 * 说明：
 * - 在“目标航向误差”“角速度积分后航向”中会频繁调用；
 * - 统一角度范围可以避免跨 ±pi 的跳变问题。
 */
inline double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief 点到线段最短距离
 *
 * 参数：
 * - (px, py): 查询点（这里通常是障碍物中心）
 * - (x1, y1) ~ (x2, y2): 线段两端（这里通常是轨迹相邻两点）
 *
 * 逻辑：
 * 1) 计算点在线段方向上的投影；
 * 2) 若投影落在线段外，取到端点距离；
 * 3) 若落在线段内，取到投影点距离。
 */
inline double point_to_segment_distance(
  double px, double py, double x1, double y1, double x2, double y2)
{
  const double vx = x2 - x1;
  const double vy = y2 - y1;
  const double wx = px - x1;
  const double wy = py - y1;

  // c1 <= 0 说明投影在起点之前。
  const double c1 = vx * wx + vy * wy;
  if (c1 <= 0.0) return std::hypot(px - x1, py - y1);

  // 线段长度平方，若极小视作退化线段。
  const double c2 = vx * vx + vy * vy;
  if (c2 <= 1e-9) return std::hypot(px - x1, py - y1);

  // c2 <= c1 说明投影在终点之后。
  if (c2 <= c1) return std::hypot(px - x2, py - y2);

  // 投影在线段内部，按比例求投影点坐标。
  const double b = c1 / c2;
  const double bx = x1 + b * vx;
  const double by = y1 + b * vy;
  return std::hypot(px - bx, py - by);
}

}  // namespace autoware::dwa_trajectory_follower

#endif  // AUTOWARE__DWA_TRAJECTORY_FOLLOWER__UTILS_HPP_
