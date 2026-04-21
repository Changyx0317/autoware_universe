// Copyright 2026
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_DIFF_DRIVE_MODEL_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_DIFF_DRIVE_MODEL_HPP_

#include <autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace autoware::freespace_planning_algorithms
{
namespace kinematic_diff_drive_model
{

static constexpr double eps = 1e-6;

inline geometry_msgs::msg::Pose getPoseFromCurvature(
  const geometry_msgs::msg::Pose & current_pose, const double curvature, const double distance)
{
  auto pose = current_pose;
  const double yaw = tf2::getYaw(current_pose.orientation);

  if (std::abs(curvature) < eps) {
    pose.position.x += distance * std::cos(yaw);
    pose.position.y += distance * std::sin(yaw);
    return pose;
  }

  const double radius = 1.0 / curvature;
  const double beta = distance * curvature;
  pose.position.x += radius * (std::sin(yaw + beta) - std::sin(yaw));
  pose.position.y += radius * (std::cos(yaw) - std::cos(yaw + beta));
  pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw + beta);
  return pose;
}

}  // namespace kinematic_diff_drive_model
}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_DIFF_DRIVE_MODEL_HPP_
