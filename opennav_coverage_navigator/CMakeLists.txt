cmake_minimum_required(VERSION 3.5)
project(opennav_coverage_navigator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(backported_bt_navigator REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wnull-dereference)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_util
  nav2_msgs
  nav2_core
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
  opennav_coverage_msgs
  backported_bt_navigator
)

include_directories(
  include
)

add_library(opennav_coverage_navigator SHARED src/coverage_navigator.cpp)
ament_target_dependencies(opennav_coverage_navigator ${dependencies})

pluginlib_export_plugin_description_file(backported_bt_navigator navigator_plugins.xml)

install(TARGETS opennav_coverage_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(opennav_coverage_navigator)
ament_export_dependencies(${dependencies})
ament_package()
