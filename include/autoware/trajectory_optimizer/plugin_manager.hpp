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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__PLUGIN_MANAGER_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__PLUGIN_MANAGER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer
{

/**
 * @brief Manages the execution of trajectory optimizer plugins
 * 
 * This class allows for flexible configuration of plugin execution order
 * and supports running the same plugin multiple times at different stages
 * of the optimization process.
 */
class PluginManager
{
public:
  /**
   * @brief Configuration for a single plugin execution
   */
  struct PluginExecutionConfig
  {
    std::string plugin_name;  // Name of the plugin to execute
    bool enabled{true};       // Whether this execution is enabled
    
    // Additional parameters for this specific execution
    std::map<std::string, double> parameters;
  };

  /**
   * @brief Constructor
   * @param node_ptr Pointer to the ROS node
   * @param time_keeper Shared time keeper for performance monitoring
   */
  PluginManager(
    rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils::TimeKeeper> & time_keeper);

  /**
   * @brief Register a plugin with the manager
   * @param name Unique identifier for the plugin
   * @param plugin Shared pointer to the plugin instance
   */
  void register_plugin(
    const std::string & name,
    std::shared_ptr<plugin::TrajectoryOptimizerPluginBase> plugin);

  /**
   * @brief Set the execution sequence for plugins
   * @param execution_sequence Vector of plugin execution configurations
   */
  void set_execution_sequence(const std::vector<PluginExecutionConfig> & execution_sequence);

  /**
   * @brief Load execution sequence from ROS parameters
   * @return true if sequence was successfully loaded
   */
  bool load_execution_sequence_from_params();

  /**
   * @brief Execute all enabled plugins in the configured sequence
   * @param trajectory_points Trajectory points to optimize
   * @param params Optimizer parameters
   */
  void execute_optimization_sequence(
    TrajectoryPoints & trajectory_points,
    const TrajectoryOptimizerParams & params);

  /**
   * @brief Get the current execution sequence
   * @return Vector of plugin execution configurations
   */
  const std::vector<PluginExecutionConfig> & get_execution_sequence() const
  {
    return execution_sequence_;
  }

  /**
   * @brief Enable or disable a specific execution step
   * @param index Index in the execution sequence
   * @param enabled Whether to enable or disable
   * @return true if the index was valid
   */
  bool set_execution_enabled(size_t index, bool enabled);

  /**
   * @brief Get a list of all registered plugin names
   * @return Vector of plugin names
   */
  std::vector<std::string> get_registered_plugin_names() const;

  /**
   * @brief Update parameters for all plugins
   * @param parameters Vector of ROS parameters
   * @return SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  rclcpp::Node * node_ptr_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  
  // Map of plugin name to plugin instance
  std::map<std::string, std::shared_ptr<plugin::TrajectoryOptimizerPluginBase>> plugins_;
  
  // Ordered sequence of plugin executions
  std::vector<PluginExecutionConfig> execution_sequence_;
  
  // Default execution sequence (used if no custom sequence is provided)
  std::vector<PluginExecutionConfig> default_sequence_;

  /**
   * @brief Initialize the default execution sequence
   */
  void initialize_default_sequence();
  
  /**
   * @brief Parse execution sequence from parameter string
   * @param sequence_param Parameter string defining the sequence
   * @return Vector of plugin execution configurations
   */
  std::vector<PluginExecutionConfig> parse_execution_sequence(const std::string & sequence_param);
};

}  // namespace autoware::trajectory_optimizer

#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__PLUGIN_MANAGER_HPP_