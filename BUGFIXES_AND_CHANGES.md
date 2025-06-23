# Bug Fixes and Changes Summary

## Bug Fixes

### 1. Fixed Null Pointer Dereference (HIGH PRIORITY)
**File**: `src/trajectory_optimizer.cpp`
**Issue**: Dereferencing `current_odometry_ptr_` and `current_acceleration_ptr_` before null check
**Fix**: Moved null checks before dereferencing pointers

### 2. Fixed Duplicate Time Keeper Initialization (MEDIUM PRIORITY) 
**Files**: 
- `src/trajectory_optimizer.cpp`
- `include/autoware/trajectory_optimizer/trajectory_optimizer.hpp`
**Issue**: `time_keeper_` was initialized twice with different publishers
**Fix**: Consolidated to single initialization and removed unused member variable

### 3. Fixed Dangerous Iterator Arithmetic (HIGH PRIORITY)
**File**: `src/utils.cpp` 
**Function**: `remove_close_proximity_points`
**Issue**: Using `std::prev(&point)` which is undefined behavior
**Fix**: Rewrote function using proper iterator-based approach

### 4. Added Division by Zero Protection (HIGH PRIORITY)
**File**: `src/utils.cpp`
**Function**: `limit_lateral_acceleration`
**Issue**: Division by `delta_time` without validation
**Fix**: Added check to skip processing if `delta_time <= 1e-6`

### 5. Fixed Out of Bounds Access (MEDIUM PRIORITY)
**File**: `src/utils.cpp`
**Function**: `add_ego_state_to_trajectory`
**Issue**: Potential out of bounds access with `clip_idx`
**Fix**: Added bounds checking and adjusted logic for correct clipping

### 6. Improved Empty Trajectory Handling (MEDIUM PRIORITY)
**File**: `src/utils.cpp`
**Function**: `apply_spline`
**Issue**: Poor handling of trajectories with 0-4 points
**Fix**: Added explicit empty check and improved logging

### 7. Null Checks for Smoother Objects (MEDIUM PRIORITY)
**Status**: Verified existing code already has proper null checks in utility functions

### 8. Fixed Type Safety Issues (LOW PRIORITY)
**File**: `src/utils.cpp`
**Issue**: Multiple unsafe `static_cast` operations without bounds checking
**Fixes**:
- Added bounds check for `closest_index` before casting
- Added bounds check for `traj_closest` before casting  
- Added overflow protection for `reserve()` calculation

## New Features

### Plugin Manager Implementation

**New Files**:
- `include/autoware/trajectory_optimizer/plugin_manager.hpp`
- `src/plugin_manager.cpp`
- `config/plugin_manager_example.yaml`

**Modified Files**:
- `include/autoware/trajectory_optimizer/trajectory_optimizer.hpp`
- `src/trajectory_optimizer.cpp`
- `CMakeLists.txt`
- `config/trajectory_optimizer.param.yaml`

**Features**:
1. **Flexible Plugin Execution**: Configure optimization sequence via parameters
2. **Multiple Executions**: Run same plugin multiple times with different configurations
3. **Dynamic Enable/Disable**: Enable or disable specific optimization steps
4. **Custom Parameters**: Pass custom parameters to specific plugin executions
5. **Runtime Reconfiguration**: Update execution sequence via ROS parameters

**Configuration Format**:
```yaml
plugin_execution_sequence: "plugin1,plugin2:disabled,plugin3,plugin1:param=value"
```

**Available Plugins**:
- `trajectory_extender`: Extends trajectory backward using ego history
- `trajectory_point_fixer`: Removes invalid/repeated points
- `eb_smoother`: Elastic Band smoother for path smoothing
- `spline_smoother`: Akima spline interpolation
- `velocity_optimizer`: Velocity profile optimization

**Example Configurations**:
```yaml
# Default (matches original hardcoded behavior)
plugin_execution_sequence: "trajectory_extender,trajectory_point_fixer,eb_smoother,spline_smoother,velocity_optimizer,spline_smoother,trajectory_point_fixer"

# Simplified sequence
plugin_execution_sequence: "trajectory_point_fixer,eb_smoother,velocity_optimizer"

# Disable specific step
plugin_execution_sequence: "trajectory_extender,trajectory_point_fixer,eb_smoother:disabled,velocity_optimizer"
```

## Testing Recommendations

1. **Unit Tests**: Update existing tests to cover new plugin manager functionality
2. **Integration Tests**: Test various plugin execution sequences
3. **Performance Tests**: Verify no performance regression with plugin manager
4. **Edge Case Tests**: Test empty trajectories, single-point trajectories, etc.

## Migration Guide

The system maintains backward compatibility. The default plugin execution sequence matches the original hardcoded behavior. To customize:

1. Update your launch file or parameter file to include `plugin_execution_sequence`
2. Refer to `config/plugin_manager_example.yaml` for configuration examples
3. Monitor logs for plugin execution order confirmation

## Future Enhancements

1. **Plugin Discovery**: Dynamic plugin loading from shared libraries
2. **Plugin Parameters**: More sophisticated per-execution parameter overrides
3. **Conditional Execution**: Execute plugins based on trajectory characteristics
4. **Performance Metrics**: Per-plugin execution time tracking and optimization