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
#include "autoware/trajectory_interpolator/trajectory_interpolator_params.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
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
  set_up_params();
  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&TrajectoryInterpolator::on_parameter, this, std::placeholders::_1));

  // interface subscriber
  trajectories_sub_ = create_subscription<Trajectories>(
    "mtr/trajectories", 1,
    std::bind(&TrajectoryInterpolator::on_traj, this, std::placeholders::_1));
  // interface publisher
  trajectories_pub_ = create_publisher<Trajectories>("~/output/trajectories", 1);
}

rcl_interfaces::msg::SetParametersResult TrajectoryInterpolator::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;
  TrajectoryInterpolatorParams params;

  updateParam<double>(parameters, "keep_last_trajectory_s", params.keep_last_trajectory_s);
  updateParam<double>(parameters, "nearest_dist_threshold_m", params.nearest_dist_threshold_m);
  updateParam<double>(parameters, "nearest_yaw_threshold_rad", params.nearest_yaw_threshold_rad);
  updateParam<double>(parameters, "target_pull_out_speed_mps", params.target_pull_out_speed_mps);
  updateParam<double>(parameters, "target_pull_out_acc_mps2", params.target_pull_out_acc_mps2);
  updateParam<double>(parameters, "max_speed_mps", params.max_speed_mps);
  updateParam<bool>(parameters, "publish_last_trajectory", params.publish_last_trajectory);
  updateParam<bool>(parameters, "keep_last_trajectory", params.keep_last_trajectory);

  params_ = params;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryInterpolator::set_up_params()
{
  using autoware::universe_utils::getOrDeclareParameter;

  params_.keep_last_trajectory_s = getOrDeclareParameter<double>(*this, "keep_last_trajectory_s");
  params_.nearest_dist_threshold_m =
    getOrDeclareParameter<double>(*this, "nearest_dist_threshold_m");
  params_.nearest_yaw_threshold_rad =
    getOrDeclareParameter<double>(*this, "nearest_yaw_threshold_rad");
  params_.target_pull_out_speed_mps =
    getOrDeclareParameter<double>(*this, "target_pull_out_speed_mps");
  params_.target_pull_out_acc_mps2 =
    getOrDeclareParameter<double>(*this, "target_pull_out_acc_mps2");
  params_.max_speed_mps = getOrDeclareParameter<double>(*this, "max_speed_mps");

  params_.publish_last_trajectory = getOrDeclareParameter<bool>(*this, "publish_last_trajectory");
  params_.keep_last_trajectory = getOrDeclareParameter<bool>(*this, "keep_last_trajectory");
}

void TrajectoryInterpolator::remove_close_proximity_points(
  TrajectoryPoints & input_trajectory_array, const double min_dist)
{
  if (std::size(input_trajectory_array) < 2) {
    return;
  }

  input_trajectory_array.erase(
    std::remove_if(
      std::next(input_trajectory_array.begin()),  // Start from second element
      input_trajectory_array.end(),
      [&](const TrajectoryPoint & point) {
        const auto prev_it = std::prev(&point);
        const auto dist = autoware::universe_utils::calcDistance2d(point, *prev_it);
        return dist < min_dist;
      }),
    input_trajectory_array.end());
}

void TrajectoryInterpolator::remove_invalid_points(TrajectoryPoints & input_trajectory)
{
  if (input_trajectory.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }
  remove_close_proximity_points(input_trajectory, 1E-2);
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
  TrajectoryPoints & input_trajectory, const double initial_motion_speed,
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
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother_->apply(
        initial_motion_speed, initial_motion_acc, input_trajectory, input_trajectory,
        debug_trajectories, false)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }
}

void TrajectoryInterpolator::interpolate_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const AccelWithCovarianceStamped & current_acceleration)
{
  // Remove overlap points and wrong orientation points
  remove_invalid_points(traj_points);

  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }

  const double & nearest_dist_threshold = params_.nearest_dist_threshold_m;
  const double & nearest_yaw_threshold = params_.nearest_yaw_threshold_rad;
  const double & target_pull_out_speed_mps = params_.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = params_.target_pull_out_acc_mps2;
  const double & max_speed_mps = params_.max_speed_mps;

  const auto current_speed = current_odometry.twist.twist.linear.x;
  const auto current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  // Set engage speed and acceleration
  if (current_speed < target_pull_out_speed_mps) {
    clamp_velocities(
      traj_points, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }
  // limit ego speed
  set_max_velocity(traj_points, static_cast<float>(max_speed_mps));
  // Smooth velocity profile
  filter_velocity(
    traj_points, initial_motion_speed, initial_motion_acc, nearest_dist_threshold,
    nearest_yaw_threshold);
  // Recalculate timestamps
  motion_utils::calculate_time_from_start(traj_points, current_odometry.pose.pose.position);

  if (traj_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }
}

void TrajectoryInterpolator::on_traj([[maybe_unused]] const Trajectories::ConstSharedPtr msg)
{
  current_odometry_ptr_ = sub_current_odometry_.takeData();
  current_acceleration_ptr_ = sub_current_acceleration_.takeData();
  previous_trajectory_ptr_ = sub_previous_trajectory_.takeData();

  const auto keep_last_trajectory_s = params_.keep_last_trajectory_s;

  auto create_output_trajectory_from_past = [&]() {
    NewTrajectory previous_trajectory;
    previous_trajectory.points = previous_trajectory_ptr_->points;
    previous_trajectory.header = msg->trajectories.front().header;
    previous_trajectory.generator_id = msg->trajectories.front().generator_id;
    return previous_trajectory;
  };

  if (previous_trajectory_ptr_ && params_.keep_last_trajectory) {
    auto current_time = now();
    auto time_diff = (rclcpp::Time(current_time) - *last_time_).seconds();
    if (time_diff < keep_last_trajectory_s) {
      Trajectories output_trajectories;
      output_trajectories.trajectories.push_back(create_output_trajectory_from_past());
      trajectories_pub_->publish(output_trajectories);
      return;
    }

    last_time_ = std::make_shared<rclcpp::Time>(now());
  }
  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_ERROR(get_logger(), "No current odometry data");
    return;
  }

  Trajectories output_trajectories = *msg;
  for (auto & trajectory : output_trajectories.trajectories) {
    interpolate_trajectory(trajectory.points, *current_odometry_ptr_, *current_acceleration_ptr_);
  }

  if (previous_trajectory_ptr_ && params_.publish_last_trajectory) {
    output_trajectories.trajectories.push_back(create_output_trajectory_from_past());
  }

  trajectories_pub_->publish(output_trajectories);
}

void TrajectoryInterpolator::clamp_velocities(
  TrajectoryPoints & input_trajectory_array, float min_velocity, float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void TrajectoryInterpolator::set_max_velocity(
  TrajectoryPoints & input_trajectory_array, const float max_velocity)
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
