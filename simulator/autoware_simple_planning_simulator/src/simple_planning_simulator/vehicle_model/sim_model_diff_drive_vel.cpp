// Copyright 2025 The Autoware Foundation.
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

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_diff_drive_vel.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDiffDriveVel::SimModelDiffDriveVel(double max_yaw_rate_rps)
: SimModelInterface(3 /* dim x */, 2 /* dim u */), max_yaw_rate_rps_(std::abs(max_yaw_rate_rps))
{
}

double SimModelDiffDriveVel::getX() { return state_(IDX::X); }

double SimModelDiffDriveVel::getY() { return state_(IDX::Y); }

double SimModelDiffDriveVel::getYaw() { return state_(IDX::YAW); }

double SimModelDiffDriveVel::getVx() { return input_(IDX_U::VX_DES); }

double SimModelDiffDriveVel::getVy() { return 0.0; }

double SimModelDiffDriveVel::getAx() { return current_ax_; }

double SimModelDiffDriveVel::getWz()
{
  return std::clamp(input_(IDX_U::WZ_DES), -max_yaw_rate_rps_, max_yaw_rate_rps_);
}

double SimModelDiffDriveVel::getSteer()
{
  // Differential drive does not use steering tire angle.
  return 0.0;
}

void SimModelDiffDriveVel::update(const double & dt)
{
  Eigen::VectorXd bounded_input = input_;
  bounded_input(IDX_U::WZ_DES) =
    std::clamp(bounded_input(IDX_U::WZ_DES), -max_yaw_rate_rps_, max_yaw_rate_rps_);
  updateRungeKutta(dt, bounded_input);

  current_ax_ = (bounded_input(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = bounded_input(IDX_U::VX_DES);
}

Eigen::VectorXd SimModelDiffDriveVel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double wz = std::clamp(input(IDX_U::WZ_DES), -max_yaw_rate_rps_, max_yaw_rate_rps_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::YAW) = wz;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
