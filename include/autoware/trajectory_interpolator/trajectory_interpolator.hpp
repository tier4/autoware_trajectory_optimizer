// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_INTERPOLATOR_HPP_
#define AUTOWARE__TRAJECTORY_INTERPOLATOR_HPP_

#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rclcpp/subscription.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory_interpolator
{

using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class TrajectoryInterpolator : public rclcpp::Node
{
public:
  explicit TrajectoryInterpolator(const rclcpp::NodeOptions & options);

private:
  void on_traj(const Trajectory::ConstSharedPtr msg);

  Trajectory interpolate_trajectory(
    const Trajectory & input_trajectory, const Odometry & current_odometry);

  static std::vector<TrajectoryPoint> clamp_negative_velocities(
    std::vector<TrajectoryPoint> & input_trajectory_array);
  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  // interface subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;

  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};

  Odometry::ConstSharedPtr current_odometry_ptr_;  // current odometry
};

}  // namespace autoware::trajectory_interpolator

#endif  // AUTOWARE__TRAJECTORY_INTERPOLATOR_HPP_
