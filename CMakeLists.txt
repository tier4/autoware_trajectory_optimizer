cmake_minimum_required(VERSION 3.22)
project(autoware_trajectory_interpolator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# control validator
ament_auto_add_library(autoware_trajectory_interpolator_component SHARED
  src/trajectory_interpolator.cpp
  src/utils.cpp
  src/trajectory_point.cpp
)
target_link_libraries(autoware_trajectory_interpolator_component)
rclcpp_components_register_node(autoware_trajectory_interpolator_component
  PLUGIN "autoware::trajectory_interpolator::TrajectoryInterpolator"
  EXECUTABLE autoware_trajectory_interpolator_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_interpolator
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_interpolator
    autoware_trajectory_interpolator_component
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
