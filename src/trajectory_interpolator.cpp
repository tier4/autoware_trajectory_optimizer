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
}

NewTrajectory TrajectoryInterpolator::interpolate_trajectory(
  const NewTrajectory & input_trajectory, [[maybe_unused]] const Odometry & current_odometry)
{
  auto traj_points = input_trajectory.points;
  // guard for invalid trajectory
  // traj_points = autoware::motion_utils::removeOverlapPoints(traj_points);
  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return input_trajectory;
  }

  clamp_negative_velocities(traj_points);

  // constexpr double nearest_dist_threshold = 2.0;
  // constexpr double nearest_yaw_threshold = 1.0;  // [rad]

  // // Arc length from the initial point to the closest point
  // const auto current_pose = current_odometry.pose.pose;

  // const size_t current_seg_idx =
  //   autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
  //     traj_points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  // const double negative_front_arclength_value = autoware::motion_utils::calcSignedArcLength(
  //   traj_points, current_pose.position, current_seg_idx, traj_points.back().pose.position, 0);
  // const auto front_arclength_value = std::fabs(negative_front_arclength_value);

  // std::vector<double> out_arclength;
  // // Step1. Resample front trajectory
  // constexpr double front_ds = 0.1;
  // for (double ds = 0.0; ds <= front_arclength_value; ds += front_ds) {
  //   out_arclength.push_back(ds);
  // }
  // if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
  //   out_arclength.back() = front_arclength_value;
  // } else {
  //   out_arclength.push_back(front_arclength_value);
  // }

  // auto output_traj = autoware::motion_utils::resampleTrajectory(
  //   autoware::motion_utils::convertToTrajectory(traj_points), out_arclength, true, true, false);

  // // output_traj =
  // //   autoware::motion_utils::resampleTrajectory(output_traj, 0.1, true, true, true, true);
  // std::cerr << "Resampled trajectory size: " << output_traj.points.size() << std::endl;
  // std::cerr << "out_arc_length: " << out_arclength.back() << std::endl;

  // int i = 0;
  // for (auto & point : output_traj.points) {
  //   std::cerr << "point(" << i++
  //             << ") x: " << point.pose.position.x - output_traj.points.front().pose.position.x
  //             << " y: " << point.pose.position.y - output_traj.points.front().pose.position.y
  //             << std::endl;
  // }

  NewTrajectory output_traj = input_trajectory;
  output_traj.points = traj_points;
  return output_traj;
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
