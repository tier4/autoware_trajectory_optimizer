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

#ifndef AUTOWARE__TRAJECTORY_INTERPOLATOR_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_INTERPOLATOR_STRUCTS_HPP_

namespace autoware::trajectory_interpolator
{

struct InitialMotion
{
  double speed_mps{0.0};
  double acc_mps2{0.0};
};
struct TrajectoryInterpolatorParams
{
  double keep_last_trajectory_s{0.0};
  double nearest_dist_threshold_m{0.0};
  double nearest_yaw_threshold_rad{0.0};
  double target_pull_out_speed_mps{0.0};
  double target_pull_out_acc_mps2{0.0};
  double max_speed_mps{0.0};
  double spline_interpolation_resolution_m{0.0};
  double backward_path_extension_m{0.0};
  bool use_akima_spline_interpolation{false};
  bool smooth_velocities{false};
  bool smooth_trajectories{false};
  bool limit_velocity{false};
  bool fix_invalid_points{false};
  bool publish_last_trajectory{false};
  bool keep_last_trajectory{false};
  bool extend_trajectory_backward{false};
};
}  // namespace autoware::trajectory_interpolator
#endif  // AUTOWARE__TRAJECTORY_INTERPOLATOR_STRUCTS_HPP_
