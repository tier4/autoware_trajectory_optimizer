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

#ifndef AUTOWARE__TRAJECTORY_INTERPOLATOR_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_INTERPOLATOR_UTILS_HPP_

#include "autoware/trajectory_interpolator/trajectory_interpolator_structs.hpp"
#include "autoware/trajectory_interpolator/trajectory_point.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include <rclcpp/logger.hpp>

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <autoware_new_planning_msgs/msg/trajectories.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include <memory>

namespace autoware::trajectory_interpolator::utils
{
using autoware::velocity_smoother::JerkFilteredSmoother;
using autoware_new_planning_msgs::msg::Trajectories;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using NewTrajectory = autoware_new_planning_msgs::msg::Trajectory;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Interpolates the given trajectory points based on trajectory length.
 *
 * @param traj_points The trajectory points to be interpolated.
 * @param params The parameters for trajectory interpolation.
 */
void apply_cubic_spline(
  TrajectoryPoints & traj_points, const TrajectoryInterpolatorParams & params);

/**
 * @brief Interpolates the given trajectory points based on the current odometry and acceleration.
 *
 * @param traj_points The trajectory points to be interpolated.
 * @param current_odometry The current odometry data.
 * @param current_acceleration The current acceleration data.
 * @param params The parameters for trajectory interpolation.
 * @param smoother The smoother to be used for filtering the trajectory.
 */
void interpolate_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const AccelWithCovarianceStamped & current_acceleration,
  const TrajectoryInterpolatorParams & params,
  const std::shared_ptr<JerkFilteredSmoother> & smoother);

/**
 * @brief Gets the logger for the trajectory interpolator.
 *
 * @return The logger instance.
 */
rclcpp::Logger get_logger();

/**
 * @brief Removes invalid points from the input trajectory.
 *
 * @param input_trajectory The trajectory points to be cleaned.
 */
void remove_invalid_points(std::vector<TrajectoryPoint> & input_trajectory);

/**
 * @brief Filters the velocity of the input trajectory based on the initial motion and parameters.
 *
 * @param input_trajectory The trajectory points to be filtered.
 * @param initial_motion_speed The initial speed for motion.
 * @param initial_motion_acc The initial acceleration for motion.
 * @param params The parameters for trajectory interpolation.
 * @param smoother The smoother to be used for filtering the trajectory.
 * @param current_odometry The current odometry data.
 */
void filter_velocity(
  std::vector<TrajectoryPoint> & input_trajectory, const double initial_motion_speed,
  const double initial_motion_acc, const TrajectoryInterpolatorParams & params,
  const std::shared_ptr<JerkFilteredSmoother> & smoother, const Odometry & current_odometry);

/**
 * @brief Clamps the velocities of the input trajectory points to the specified minimum values.
 *
 * @param input_trajectory_array The trajectory points to be clamped.
 * @param min_velocity The minimum velocity to be clamped.
 * @param min_acceleration The minimum acceleration to be clamped.
 */
void clamp_velocities(
  std::vector<TrajectoryPoint> & input_trajectory_array, float min_velocity,
  float min_acceleration);

/**
 * @brief Sets the maximum velocity for the input trajectory points.
 *
 * @param input_trajectory_array The trajectory points to be updated.
 * @param max_velocity The maximum velocity to be set.
 */
void set_max_velocity(
  std::vector<TrajectoryPoint> & input_trajectory_array, const float max_velocity);

/**
 * @brief Removes points from the input trajectory that are too close to each other.
 *
 * @param input_trajectory_array The trajectory points to be cleaned.
 * @param min_dist The minimum distance between points.
 */
void remove_close_proximity_points(
  std::vector<TrajectoryPoint> & input_trajectory_array, const double min_dist = 1E-2);

};  // namespace autoware::trajectory_interpolator::utils

#endif  // AUTOWARE__TRAJECTORY_INTERPOLATOR_UTILS_HPP_
