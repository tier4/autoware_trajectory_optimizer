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

#include "autoware/prediction_to_trajectory_converter/prediction_to_trajectory_converter.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <iostream>
#include <vector>

namespace autoware::prediction_to_trajectory_converter
{

PredictionToTrajectory::PredictionToTrajectory(const rclcpp::NodeOptions & options)
: ConverterBase("prediction_to_trajectory_converter", options)
{
  // Retrieve parameters from the base class
  const auto input_topic = this->get_parameter("input_topic").as_string();
  const auto output_topic = this->get_parameter("output_topic").as_string();

  // Define a custom QoS profile for the subscription
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  // Override the subscription with the new QoS profile
  sub_ = this->create_subscription<PredictedObjects>(
    input_topic, qos_profile,
    std::bind(&PredictionToTrajectory::process, this, std::placeholders::_1));

  // Define a custom QoS profile for the publisher (e.g., Reliable or BestEffort)
  rclcpp::QoS pub_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos_profile.keep_last(10);  // Configure depth as needed

  // Override the publisher with the new QoS profile
  pub_ = this->create_publisher<Trajectory>(output_topic, pub_qos_profile);
}

void PredictionToTrajectory::process([[maybe_unused]] const PredictedObjects::ConstSharedPtr msg)
{
  if (msg->objects.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty predicted objects message");
    return;
  }
  const auto & header = msg->header;
  const auto & ego_object = msg->objects[0];
  Trajectory trajectory;
  trajectory.header = header;
  auto max_distance = 0.0;
  for (const auto & predicted_path : ego_object.kinematics.predicted_paths) {
    std::vector<TrajectoryPoint> trajectory_points;
    for (const auto & point : predicted_path.path) {
      TrajectoryPoint trajectory_point;
      trajectory_point.pose = point;
      trajectory_point.longitudinal_velocity_mps =
        static_cast<float>(ego_object.kinematics.initial_twist_with_covariance.twist.linear.x);
      trajectory_point.acceleration_mps2 = 0.0;
      trajectory_points.push_back(trajectory_point);
    }
    const auto distance = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, 0, trajectory_points.size() - 1);
    if (distance > max_distance) {
      max_distance = distance;
      trajectory.points = trajectory_points;
    }
  }
  trajectory.header = header;
  trajectory.header.stamp = this->now();
  pub_->publish(trajectory);
}
}  // namespace autoware::prediction_to_trajectory_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::prediction_to_trajectory_converter::PredictionToTrajectory)
