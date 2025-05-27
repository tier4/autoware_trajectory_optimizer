// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_interpolator/trajectory_optimizer_plugins/trajectory_velocity_limiter.hpp"

#include "autoware/trajectory_interpolator/utils.hpp"

namespace autoware::trajectory_interpolator::plugin
{
void TrajectoryVelocityLimiter::optimize_trajectory(
  TrajectoryPoints & traj_points, [[maybe_unused]] const TrajectoryInterpolatorParams & params)
{
  const auto & current_odometry = params.current_odometry;
  const auto & current_acceleration = params.current_acceleration;
  const auto & current_speed = current_odometry.twist.twist.linear.x;
  const auto & current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  const double & target_pull_out_speed_mps = params.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = params.target_pull_out_acc_mps2;
  const double & max_speed_mps = params.max_speed_mps;

  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  // Set engage speed and acceleration
  if (params.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    utils::clamp_velocities(
      traj_points, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }
  // Limit ego speed
  if (params.limit_speed) {
    utils::set_max_velocity(traj_points, static_cast<float>(max_speed_mps));
  }
}

void TrajectoryVelocityLimiter::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectoryVelocityLimiter::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_interpolator::plugin
