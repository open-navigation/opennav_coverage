cmake_minimum_required(VERSION 3.5)
project(opennav_row_coverage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(GDAL REQUIRED)
find_package(opennav_coverage REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)
set(executable_name opennav_row_coverage)
set(library_name ${executable_name}_core)

set(dependencies
  nav2_util
  nav2_ros_common
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2_ros
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  opennav_coverage
  opennav_coverage_msgs
)

# Modern CMake targets for the dependencies above. Rolling removed the
# ament_target_dependencies macro, so we link these targets directly; this also
# works on jazzy, which still exports the same targets. Message/interface
# packages expose a ${pkg_TARGETS} variable, regular packages a pkg::pkg target.
# opennav_coverage uses the classic ament_export_libraries (no namespaced
# target), so it is consumed via ${opennav_coverage_LIBRARIES}/_INCLUDE_DIRS
# below. tf2_ros and the message packages are pulled in transitively by
# find_package(opennav_coverage) through its ament_export_dependencies.
set(dep_targets
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${opennav_coverage_msgs_TARGETS}
)

add_library(${library_name} SHARED
  src/row_swath_generator.cpp
  src/row_coverage_server.cpp
)

# Fields2Cover exposes a different target name depending on the install method
# (source install: Fields2Cover; ROS/apt package: Fields2Cover::Fields2Cover).
if(TARGET Fields2Cover::Fields2Cover)
  set(FIELDS2COVER_LIBRARIES Fields2Cover::Fields2Cover)
else()
  set(FIELDS2COVER_LIBRARIES Fields2Cover)
endif()

target_link_libraries(${library_name} PUBLIC
  ${dep_targets}
  ${opennav_coverage_LIBRARIES}
  ${FIELDS2COVER_LIBRARIES}
  tinyxml2::tinyxml2
  GDAL::GDAL
)
target_include_directories(${library_name} PUBLIC ${opennav_coverage_INCLUDE_DIRS})

# Treat Fields2Cover headers as SYSTEM so their own warnings do not break this
# project's -Werror build. Uses the config-provided include variable (path- and
# target-name-agnostic; apt /opt/ros or source /usr/local).
if(Fields2Cover_INCLUDE_DIR)
  target_include_directories(${library_name} SYSTEM PUBLIC ${Fields2Cover_INCLUDE_DIR})
endif()

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "opennav_row_coverage::RowCoverageServer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
