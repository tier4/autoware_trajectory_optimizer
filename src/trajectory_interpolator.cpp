// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory_interpolator/trajectory_interpolator.hpp"

#include "autoware/motion_utils/resample/resample.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

namespace autoware::trajectory_interpolator
{

TrajectoryInterpolator::TrajectoryInterpolator(const rclcpp::NodeOptions & options)
: Node("trajectory_interpolator", options)
{
  // interface subscriber
  trajectories_sub_ = create_subscription<Trajectories>(
    "mtr/trajectories", 10,
    std::bind(&TrajectoryInterpolator::on_traj, this, std::placeholders::_1));
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("smoothed/mtr/trajectory", 5);
  trajectories_pub_ = create_publisher<Trajectories>("smoothed/mtr/trajectories", 5);

  // create time_keeper and its publisher
  // NOTE: This has to be called before setupSmoother to pass the time_keeper to the smoother.
  debug_processing_time_detail_ = create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(debug_processing_time_detail_);

  // const auto vehicle_info =
  // autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  constexpr double wheelbase = 2.74;  // vehicle_info.wheel_base_m;
  smoother_ = std::make_shared<JerkFilteredSmoother>(*this, time_keeper_);
  smoother_->setWheelBase(wheelbase);
}

NewTrajectory TrajectoryInterpolator::interpolate_trajectory(
  const NewTrajectory & input_trajectory, [[maybe_unused]] const Odometry & current_odometry)
{
  auto traj_points = input_trajectory.points;
  // guard for invalid trajectory
  traj_points = autoware::motion_utils::removeOverlapPoints(traj_points);
  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return input_trajectory;
  }

  clamp_negative_velocities(traj_points);

  // constexpr double nearest_dist_threshold = 2.0;
  // constexpr double nearest_yaw_threshold = 1.0;  // [rad]

  // // Resample trajectory with ego-velocity based interval distance
  // auto traj_resampled = smoother_->resampleTrajectory(
  //   traj_points, current_odometry_ptr_->twist.twist.linear.x, current_odometry_ptr_->pose.pose,
  //   nearest_dist_threshold, nearest_yaw_threshold);

  // const size_t traj_resampled_closest =
  //   autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
  //     traj_resampled, current_odometry_ptr_->pose.pose, nearest_dist_threshold,
  //     nearest_yaw_threshold);

  // // Clip trajectory from closest point
  // std::vector<TrajectoryPoint> clipped;
  // clipped.insert(
  //   clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());

  // // Set maximum acceleration before applying smoother. Depends on acceleration request from
  // // external velocity limit
  // const double smoother_max_acceleration = get_parameter("normal.max_acc").as_double();
  // const double smoother_max_jerk = get_parameter("normal.max_jerk").as_double();
  // smoother_->setMaxAccel(smoother_max_acceleration);
  // smoother_->setMaxJerk(smoother_max_jerk);

  // auto first_point_speed = traj_points.front().longitudinal_velocity_mps;
  // auto first_point_acc = traj_points.front().acceleration_mps2;

  // std::vector<std::vector<TrajectoryPoint> > debug_trajectories;
  // std::vector<TrajectoryPoint> traj_smoothed;
  // if (!smoother_->apply(
  //       first_point_speed, first_point_acc, clipped, traj_smoothed, debug_trajectories, false)) {
  //   RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  // }

  NewTrajectory output_new_traj = input_trajectory;
  output_new_traj.points = traj_points;

  for (auto & point : output_new_traj.points) {
    point.longitudinal_velocity_mps += 0.1;
    if (current_odometry_ptr_->twist.twist.linear.x < 2.0) point.acceleration_mps2 = 1.0;
  }

  return output_new_traj;
}

void TrajectoryInterpolator::on_traj([[maybe_unused]] const Trajectories::ConstSharedPtr msg)
{
  current_odometry_ptr_ = sub_current_odometry_.takeData();
  if (!current_odometry_ptr_) {
    RCLCPP_ERROR(get_logger(), "No current odometry data");
    return;
  }

  Trajectories output_trajectories = *msg;
  output_trajectories.trajectories.clear();
  for (auto & trajectory : msg->trajectories) {
    auto out_trajectory = interpolate_trajectory(trajectory, *current_odometry_ptr_);
    output_trajectories.trajectories.push_back(out_trajectory);
  }

  trajectories_pub_->publish(output_trajectories);

  Trajectory sample_trajectory;
  sample_trajectory.points = output_trajectories.trajectories.front().points;
  sample_trajectory.header = output_trajectories.trajectories.front().header;
  traj_pub_->publish(sample_trajectory);
}

std::vector<TrajectoryPoint> TrajectoryInterpolator::clamp_negative_velocities(
  std::vector<TrajectoryPoint> & input_trajectory_array)
{
  std::vector<TrajectoryPoint> output_trajectory;
  for (const auto & point : input_trajectory_array) {
    TrajectoryPoint traj_point = point;
    if (traj_point.longitudinal_velocity_mps < 0.0) {
      traj_point.longitudinal_velocity_mps = 0.0;
    }
    output_trajectory.push_back(traj_point);
  }
  return output_trajectory;
}

}  // namespace autoware::trajectory_interpolator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_interpolator::TrajectoryInterpolator)
