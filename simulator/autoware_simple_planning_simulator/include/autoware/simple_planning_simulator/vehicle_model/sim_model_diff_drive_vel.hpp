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

#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DIFF_DRIVE_VEL_HPP_
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DIFF_DRIVE_VEL_HPP_

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

#include <Eigen/Core>

namespace autoware::simulator::simple_planning_simulator
{

class SimModelDiffDriveVel : public SimModelInterface
{
public:
  explicit SimModelDiffDriveVel(double max_yaw_rate_rps);
  ~SimModelDiffDriveVel() = default;

private:
  enum IDX { X = 0, Y, YAW };
  enum IDX_U {
    VX_DES = 0,
    WZ_DES,
  };

  const double max_yaw_rate_rps_;
  double prev_vx_ = 0.0;
  double current_ax_ = 0.0;

  double getX() override;
  double getY() override;
  double getYaw() override;
  double getVx() override;
  double getVy() override;
  double getAx() override;
  double getWz() override;
  double getSteer() override;
  void update(const double & dt) override;
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;
};

}  // namespace autoware::simulator::simple_planning_simulator

#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DIFF_DRIVE_VEL_HPP_
