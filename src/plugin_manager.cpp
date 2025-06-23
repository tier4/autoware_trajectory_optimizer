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

#include "autoware/trajectory_optimizer/plugin_manager.hpp"

#include <rclcpp/logging.hpp>

#include <sstream>

namespace autoware::trajectory_optimizer
{

PluginManager::PluginManager(
  rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils::TimeKeeper> & time_keeper)
: node_ptr_(node_ptr), time_keeper_(time_keeper)
{
  initialize_default_sequence();
}

void PluginManager::register_plugin(
  const std::string & name,
  std::shared_ptr<plugin::TrajectoryOptimizerPluginBase> plugin)
{
  if (plugins_.find(name) != plugins_.end()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Plugin with name '%s' already registered, overwriting", name.c_str());
  }
  
  plugins_[name] = plugin;
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Registered plugin: %s", name.c_str());
}

void PluginManager::set_execution_sequence(
  const std::vector<PluginExecutionConfig> & execution_sequence)
{
  execution_sequence_ = execution_sequence;
  
  // Validate that all plugins in the sequence are registered
  for (const auto & config : execution_sequence_) {
    if (plugins_.find(config.plugin_name) == plugins_.end()) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Plugin '%s' in execution sequence is not registered", config.plugin_name.c_str());
    }
  }
}

bool PluginManager::load_execution_sequence_from_params()
{
  // Try to load custom execution sequence from parameters
  auto sequence_param = node_ptr_->declare_parameter<std::string>(
    "plugin_execution_sequence", "");
  
  if (sequence_param.empty()) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "No custom plugin execution sequence found, using default");
    execution_sequence_ = default_sequence_;
    return true;
  }
  
  try {
    execution_sequence_ = parse_execution_sequence(sequence_param);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Loaded custom plugin execution sequence with %zu steps", execution_sequence_.size());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Failed to parse plugin execution sequence: %s", e.what());
    execution_sequence_ = default_sequence_;
    return false;
  }
}

void PluginManager::execute_optimization_sequence(
  TrajectoryPoints & trajectory_points,
  const TrajectoryOptimizerParams & params)
{
  autoware_utils::ScopedTimeTrack st("plugin_manager_execution", *time_keeper_);
  
  for (size_t i = 0; i < execution_sequence_.size(); ++i) {
    const auto & config = execution_sequence_[i];
    
    if (!config.enabled) {
      RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "Skipping disabled execution step %zu: %s", i, config.plugin_name.c_str());
      continue;
    }
    
    auto plugin_it = plugins_.find(config.plugin_name);
    if (plugin_it == plugins_.end()) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Plugin '%s' not found, skipping", config.plugin_name.c_str());
      continue;
    }
    
    try {
      autoware_utils::ScopedTimeTrack plugin_st(
        config.plugin_name + "_step_" + std::to_string(i), *time_keeper_);
      
      RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "Executing plugin step %zu: %s", i, config.plugin_name.c_str());
      
      // Create a copy of params with any custom parameters for this execution
      TrajectoryOptimizerParams exec_params = params;
      // TODO: Apply custom parameters from config.parameters if needed
      
      plugin_it->second->optimize_trajectory(trajectory_points, exec_params);
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Error executing plugin '%s': %s", config.plugin_name.c_str(), e.what());
    }
  }
}

bool PluginManager::set_execution_enabled(size_t index, bool enabled)
{
  if (index >= execution_sequence_.size()) {
    return false;
  }
  
  execution_sequence_[index].enabled = enabled;
  return true;
}

std::vector<std::string> PluginManager::get_registered_plugin_names() const
{
  std::vector<std::string> names;
  names.reserve(plugins_.size());
  
  for (const auto & [name, plugin] : plugins_) {
    names.push_back(name);
  }
  
  return names;
}

rcl_interfaces::msg::SetParametersResult PluginManager::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  // Forward parameters to all plugins
  for (const auto & [name, plugin] : plugins_) {
    auto plugin_result = plugin->on_parameter(parameters);
    if (!plugin_result.successful) {
      result.successful = false;
      result.reason += "Plugin " + name + ": " + plugin_result.reason + "; ";
    }
  }
  
  // Check for plugin sequence parameter updates
  for (const auto & param : parameters) {
    if (param.get_name() == "plugin_execution_sequence") {
      try {
        execution_sequence_ = parse_execution_sequence(param.as_string());
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Updated plugin execution sequence");
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason += "Failed to parse execution sequence: " + std::string(e.what());
      }
    }
  }
  
  if (result.successful) {
    result.reason = "success";
  }
  
  return result;
}

void PluginManager::initialize_default_sequence()
{
  // This matches the current hardcoded sequence in trajectory_optimizer.cpp
  default_sequence_ = {
    {.plugin_name = "trajectory_extender", .enabled = true},
    {.plugin_name = "trajectory_point_fixer", .enabled = true},
    {.plugin_name = "eb_smoother", .enabled = true},
    {.plugin_name = "spline_smoother", .enabled = true},
    {.plugin_name = "velocity_optimizer", .enabled = true},
    {.plugin_name = "spline_smoother", .enabled = true},  // Second run
    {.plugin_name = "trajectory_point_fixer", .enabled = true}  // Second run
  };
}

std::vector<PluginManager::PluginExecutionConfig> PluginManager::parse_execution_sequence(
  const std::string & sequence_param)
{
  std::vector<PluginExecutionConfig> sequence;
  
  // Parse format: "plugin1,plugin2:disabled,plugin1,plugin3:param1=1.0:param2=2.0"
  std::stringstream ss(sequence_param);
  std::string step;
  
  while (std::getline(ss, step, ',')) {
    if (step.empty()) continue;
    
    PluginExecutionConfig config;
    
    // Parse step format: "plugin_name[:disabled][:param=value]..."
    std::stringstream step_ss(step);
    std::string token;
    bool first_token = true;
    
    while (std::getline(step_ss, token, ':')) {
      if (first_token) {
        config.plugin_name = token;
        first_token = false;
      } else if (token == "disabled") {
        config.enabled = false;
      } else {
        // Parse parameter format: "param=value"
        size_t equals_pos = token.find('=');
        if (equals_pos != std::string::npos) {
          std::string param_name = token.substr(0, equals_pos);
          std::string param_value = token.substr(equals_pos + 1);
          try {
            config.parameters[param_name] = std::stod(param_value);
          } catch (const std::exception &) {
            RCLCPP_WARN(
              node_ptr_->get_logger(),
              "Failed to parse parameter value: %s", token.c_str());
          }
        }
      }
    }
    
    if (!config.plugin_name.empty()) {
      sequence.push_back(config);
    }
  }
  
  if (sequence.empty()) {
    throw std::runtime_error("Parsed execution sequence is empty");
  }
  
  return sequence;
}

}  // namespace autoware::trajectory_optimizer