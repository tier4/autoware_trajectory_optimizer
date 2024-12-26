cmake_minimum_required(VERSION 3.22)
project(autoware_prediction_to_trajectory_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()


# control validator
ament_auto_add_library(autoware_prediction_to_trajectory_converter_component SHARED
  include/autoware/prediction_to_trajectory_converter/prediction_to_trajectory_converter.hpp
  src/prediction_to_trajectory_converter.cpp
)
target_link_libraries(autoware_prediction_to_trajectory_converter_component)
rclcpp_components_register_node(autoware_prediction_to_trajectory_converter_component
  PLUGIN "autoware::prediction_to_trajectory_converter::PredictionToTrajectory"
  EXECUTABLE autoware_prediction_to_trajectory_converter_node
)

# if(BUILD_TESTING)
#   file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
#   ament_add_gtest(test_autoware_prediction_to_trajectory_converter
#     ${TEST_SOURCES}
#   )
#   target_link_libraries(test_autoware_prediction_to_trajectory_converter autoware_prediction_to_trajectory_converter_component)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
