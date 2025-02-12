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

#include <autoware_new_planning_msgs/msg/detail/trajectories__struct.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>

namespace autoware::trajectory_interpolator
{

TrajectoryInterpolator::TrajectoryInterpolator(const rclcpp::NodeOptions & options)
: Node("trajectory_interpolator", options)
{
  // interface subscriber
  trajectories_sub_ = create_subscription<Trajectories>(
    "mtr/trajectories", 1,
    std::bind(&TrajectoryInterpolator::on_traj, this, std::placeholders::_1));
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  trajectories_pub_ = create_publisher<Trajectories>("~/output/trajectories", 1);

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
  // // Set maximum acceleration before applying smoother. Depends on acceleration request from
  // // external velocity limit
  const double smoother_max_acceleration = get_parameter("normal.max_acc").as_double();
  const double smoother_max_jerk = get_parameter("normal.max_jerk").as_double();
  smoother_->setMaxAccel(smoother_max_acceleration);
  smoother_->setMaxJerk(smoother_max_jerk);
  last_time_ = std::make_shared<rclcpp::Time>(now());
}

void TrajectoryInterpolator::remove_invalid_points(std::vector<TrajectoryPoint> & input_trajectory)
{
  input_trajectory = autoware::motion_utils::removeOverlapPoints(input_trajectory);
  if (input_trajectory.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }

  // const bool is_driving_forward =
  //   autoware::universe_utils::isDrivingForward(input_trajectory.at(0), input_trajectory.at(1));
  const bool is_driving_forward = true;
  autoware::motion_utils::insertOrientation(input_trajectory, is_driving_forward);

  autoware::motion_utils::removeFirstInvalidOrientationPoints(input_trajectory);
  size_t previous_size{input_trajectory.size()};
  do {
    previous_size = input_trajectory.size();
    // Set the azimuth orientation to the next point at each point
    autoware::motion_utils::insertOrientation(input_trajectory, is_driving_forward);
    // Use azimuth orientation to remove points in reverse order
    autoware::motion_utils::removeFirstInvalidOrientationPoints(input_trajectory);
  } while (previous_size != input_trajectory.size());
}

void TrajectoryInterpolator::filter_velocity(
  std::vector<TrajectoryPoint> & input_trajectory, const double initial_motion_speed,
  const double initial_motion_acc, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  // Lateral acceleration limit
  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;

  input_trajectory = smoother_->applyLateralAccelerationFilter(
    input_trajectory, initial_motion_speed, initial_motion_acc, enable_smooth_limit,
    use_resampling);

  // Steering angle rate limit (Note: set use_resample = false since it is resampled above)
  input_trajectory = smoother_->applySteeringRateLimit(input_trajectory, false);
  // Resample trajectory with ego-velocity based interval distance

  input_trajectory = smoother_->resampleTrajectory(
    input_trajectory, initial_motion_speed, current_odometry_ptr_->pose.pose,
    nearest_dist_threshold, nearest_yaw_threshold);

  if (input_trajectory.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }

  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_trajectory, current_odometry_ptr_->pose.pose, nearest_dist_threshold,
    nearest_yaw_threshold);

  // // Clip trajectory from closest point
  std::vector<TrajectoryPoint> clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() +
      static_cast<std::vector<TrajectoryPoint>::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  std::vector<std::vector<TrajectoryPoint> > debug_trajectories;
  if (!smoother_->apply(
        initial_motion_speed, initial_motion_acc, input_trajectory, input_trajectory,
        debug_trajectories, false)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }
}

NewTrajectory TrajectoryInterpolator::interpolate_trajectory(
  const NewTrajectory & input_trajectory, [[maybe_unused]] const Odometry & current_odometry)
{
  auto traj_points = input_trajectory.points;

  // Remove overlap points and wrong orientation points
  remove_invalid_points(traj_points);

  // TODO(Daniel): Add parameters to the node
  constexpr double nearest_dist_threshold = 0.5;
  constexpr double nearest_yaw_threshold = 1.0;  // [rad]

  constexpr double target_pull_out_speed_mps = 1.0;
  constexpr double target_pull_out_acc_mps2 = 1.0;

  constexpr double max_speed_mps = 5.0;

  const auto current_speed = current_odometry.twist.twist.linear.x;
  const auto current_acceleration = current_acceleration_ptr_->accel.accel.linear.x;
  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc =
    (current_speed > target_pull_out_speed_mps) ? current_acceleration : target_pull_out_acc_mps2;

  // Set engage speed and acceleration
  if (current_speed < target_pull_out_speed_mps) {
    clamp_velocities(traj_points, initial_motion_speed, initial_motion_acc);
  }
  // limit ego speed
  set_max_velocity(traj_points, max_speed_mps);
  // Smooth velocity profile
  filter_velocity(
    traj_points, initial_motion_speed, initial_motion_acc, nearest_dist_threshold,
    nearest_yaw_threshold);
  // Recalculate timestamps
  motion_utils::calculate_time_from_start(traj_points, current_odometry.pose.pose.position);

  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return input_trajectory;
  }

  NewTrajectory output_new_traj = input_trajectory;
  output_new_traj.points = traj_points;
  return output_new_traj;
}

void TrajectoryInterpolator::on_traj([[maybe_unused]] const Trajectories::ConstSharedPtr msg)
{
  current_odometry_ptr_ = sub_current_odometry_.takeData();
  current_acceleration_ptr_ = sub_current_acceleration_.takeData();
  previous_trajectory_ptr_ = sub_previous_trajectory_.takeData();

  if (previous_trajectory_ptr_) {
    auto current_time = now();
    auto time_diff = (rclcpp::Time(current_time) - *last_time_).seconds();
    if (time_diff < 0.5) {
      Trajectories output_trajectories;
      output_trajectories.generator_info = msg->generator_info;
      NewTrajectory previous_trajectory;
      previous_trajectory.points = previous_trajectory_ptr_->points;
      previous_trajectory.header = msg->trajectories.front().header;
      previous_trajectory.generator_id = msg->trajectories.front().generator_id;
      output_trajectories.trajectories.push_back(previous_trajectory);
      trajectories_pub_->publish(output_trajectories);
      // traj_pub_->publish(*previous_trajectory_ptr_);
      return;
    }

    last_time_ = std::make_shared<rclcpp::Time>(now());
  }
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

  if (previous_trajectory_ptr_) {
    NewTrajectory previous_trajectory;
    // Here we are just copying from the input, in the future the prev traj should have its own

    previous_trajectory.generator_id = output_trajectories.trajectories.front().generator_id;
    previous_trajectory.header = previous_trajectory_ptr_->header;
    previous_trajectory.points = previous_trajectory_ptr_->points;
    output_trajectories.trajectories.push_back(previous_trajectory);
  }

  trajectories_pub_->publish(output_trajectories);

  Trajectory sample_trajectory;
  sample_trajectory.points = output_trajectories.trajectories.front().points;
  sample_trajectory.header = output_trajectories.trajectories.front().header;
  traj_pub_->publish(sample_trajectory);
}

void TrajectoryInterpolator::clamp_velocities(
  std::vector<TrajectoryPoint> & input_trajectory_array, float min_velocity, float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void TrajectoryInterpolator::set_max_velocity(
  std::vector<TrajectoryPoint> & input_trajectory_array, const float max_velocity)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [max_velocity](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::min(point.longitudinal_velocity_mps, max_velocity);
    });
}

}  // namespace autoware::trajectory_interpolator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_interpolator::TrajectoryInterpolator)
