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
#include "autoware/trajectory_interpolator/utils.hpp"
#include "autoware_utils/ros/parameter.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/update_param.hpp>
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
  debug_processing_time_detail_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_);
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  double wheelbase = vehicle_info.wheel_base_m;  // vehicle_info.wheel_base_m;
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
  // debug time keeper
  debug_processing_time_detail_pub_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
}

rcl_interfaces::msg::SetParametersResult TrajectoryInterpolator::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto params = params_;

  update_param<double>(parameters, "keep_last_trajectory_s", params.keep_last_trajectory_s);
  update_param<double>(parameters, "nearest_dist_threshold_m", params.nearest_dist_threshold_m);
  update_param<double>(parameters, "nearest_yaw_threshold_rad", params.nearest_yaw_threshold_rad);
  update_param<double>(parameters, "target_pull_out_speed_mps", params.target_pull_out_speed_mps);
  update_param<double>(parameters, "target_pull_out_acc_mps2", params.target_pull_out_acc_mps2);
  update_param<double>(parameters, "max_speed_mps", params.max_speed_mps);
  update_param<double>(
    parameters, "spline_interpolation_resolution_m", params.spline_interpolation_resolution_m);
  update_param<double>(parameters, "backward_path_extension_m", params.backward_path_extension_m);
  update_param<bool>(
    parameters, "use_akima_spline_interpolation", params.use_akima_spline_interpolation);
  update_param<bool>(parameters, "smooth_velocities", params.smooth_velocities);
  update_param<bool>(parameters, "publish_last_trajectory", params.publish_last_trajectory);
  update_param<bool>(parameters, "keep_last_trajectory", params.keep_last_trajectory);
  update_param<bool>(parameters, "extend_trajectory_backward", params.extend_trajectory_backward);

  params_ = params;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryInterpolator::set_up_params()
{
  using autoware_utils::get_or_declare_parameter;

  params_.keep_last_trajectory_s =
    get_or_declare_parameter<double>(*this, "keep_last_trajectory_s");
  params_.nearest_dist_threshold_m =
    get_or_declare_parameter<double>(*this, "nearest_dist_threshold_m");
  params_.nearest_yaw_threshold_rad =
    get_or_declare_parameter<double>(*this, "nearest_yaw_threshold_rad");
  params_.target_pull_out_speed_mps =
    get_or_declare_parameter<double>(*this, "target_pull_out_speed_mps");
  params_.target_pull_out_acc_mps2 =
    get_or_declare_parameter<double>(*this, "target_pull_out_acc_mps2");
  params_.max_speed_mps = get_or_declare_parameter<double>(*this, "max_speed_mps");
  params_.spline_interpolation_resolution_m =
    get_or_declare_parameter<double>(*this, "spline_interpolation_resolution_m");
  params_.backward_path_extension_m =
    get_or_declare_parameter<double>(*this, "backward_path_extension_m");
  params_.use_akima_spline_interpolation =
    get_or_declare_parameter<bool>(*this, "use_akima_spline_interpolation");
  params_.smooth_velocities = get_or_declare_parameter<bool>(*this, "smooth_velocities");
  params_.publish_last_trajectory =
    get_or_declare_parameter<bool>(*this, "publish_last_trajectory");
  params_.keep_last_trajectory = get_or_declare_parameter<bool>(*this, "keep_last_trajectory");
  params_.extend_trajectory_backward =
    get_or_declare_parameter<bool>(*this, "extend_trajectory_backward");
}

void TrajectoryInterpolator::on_traj([[maybe_unused]] const Trajectories::ConstSharedPtr msg)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  auto create_output_trajectory_from_past = [&]() {
    NewTrajectory previous_trajectory;
    previous_trajectory.points = previous_trajectory_ptr_->points;
    previous_trajectory.header = previous_trajectory_ptr_->header;
    previous_trajectory.header.stamp = now();
    previous_trajectory.generator_id = msg->trajectories.front().generator_id;
    previous_trajectory.score = 1.0;
    motion_utils::calculate_time_from_start(
      previous_trajectory.points, current_odometry_ptr_->pose.pose.position);
    return previous_trajectory;
  };
  previous_trajectory_ptr_ = sub_previous_trajectory_.take_data();
  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();

  const auto keep_last_trajectory_s = params_.keep_last_trajectory_s;
  const auto keep_last_trajectory = params_.keep_last_trajectory;

  if (previous_trajectory_ptr_ && keep_last_trajectory) {
    auto current_time = now();
    auto time_diff = (rclcpp::Time(current_time) - *last_time_).seconds();
    if (time_diff < keep_last_trajectory_s) {
      Trajectories output_trajectories = *msg;
      output_trajectories.trajectories.clear();
      output_trajectories.trajectories.push_back(create_output_trajectory_from_past());
      trajectories_pub_->publish(output_trajectories);
      return;
    }
  }
  last_time_ = std::make_shared<rclcpp::Time>(now());

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_ERROR(get_logger(), "No odometry or acceleration data");
    return;
  }

  Trajectories output_trajectories = *msg;
  for (auto & trajectory : output_trajectories.trajectories) {
    utils::interpolate_trajectory(
      trajectory.points, previous_trajectory_ptr_, *current_odometry_ptr_,
      *current_acceleration_ptr_, params_, smoother_);
  }

  if (previous_trajectory_ptr_ && params_.publish_last_trajectory) {
    output_trajectories.trajectories.push_back(create_output_trajectory_from_past());
  }

  trajectories_pub_->publish(output_trajectories);
}

}  // namespace autoware::trajectory_interpolator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_interpolator::TrajectoryInterpolator)
