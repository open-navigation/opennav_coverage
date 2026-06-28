cmake_minimum_required(VERSION 3.5)
project(opennav_coverage_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_util
  nav2_bt_navigator
  nav2_msgs
  nav2_core
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
  opennav_coverage_msgs
  behaviortree_cpp
)

# Modern CMake targets for the dependencies above. Rolling/Lyrical removed the
# ament_target_dependencies macro, so we link these targets directly; this also
# works on jazzy. Message packages expose a ${pkg_TARGETS} variable, regular
# packages a pkg::pkg target. (nav2_bt_navigator is not used by the sources, so
# it is exported as a dependency but not linked here.)
set(dep_targets
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav2_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_behavior_tree::nav2_behavior_tree
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${opennav_coverage_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
)

add_library(opennav_compute_complete_coverage_action_bt_node SHARED src/compute_complete_coverage_path.cpp)
list(APPEND plugin_libs opennav_compute_complete_coverage_action_bt_node)
add_library(opennav_cancel_complete_coverage_action_bt_node SHARED src/cancel_complete_coverage_path.cpp)
list(APPEND plugin_libs opennav_cancel_complete_coverage_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} PUBLIC ${dep_targets})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
