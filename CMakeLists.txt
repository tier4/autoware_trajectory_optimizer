cmake_minimum_required(VERSION 3.22)
project(autoware_trajectory_interpolator)

find_package(autoware_cmake REQUIRED)
autoware_package()


# control validator
ament_auto_add_library(autoware_trajectory_interpolator_component SHARED
  include/autoware/trajectory_interpolator/trajectory_interpolator.hpp
  src/trajectory_interpolator.cpp
)
target_link_libraries(autoware_trajectory_interpolator_component)
rclcpp_components_register_node(autoware_trajectory_interpolator_component
  PLUGIN "autoware::trajectory_interpolator::PredictionToTrajectory"
  EXECUTABLE autoware_trajectory_interpolator_node
)

# if(BUILD_TESTING)
#   file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
#   ament_add_gtest(test_autoware_trajectory_interpolator
#     ${TEST_SOURCES}
#   )
#   target_link_libraries(test_autoware_trajectory_interpolator autoware_trajectory_interpolator_component)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
