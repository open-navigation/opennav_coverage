cmake_minimum_required(VERSION 3.5)
project(opennav_coverage)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(GDAL REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)

set(executable_name opennav_coverage)
set(library_name ${executable_name}_core)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  nav2_util
  nav2_ros_common
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  opennav_coverage_msgs
)

set(dep_targets
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  tf2_ros::tf2_ros
  ${opennav_coverage_msgs_TARGETS}
)

add_library(${library_name} SHARED
  src/coverage_server.cpp
  src/headland_generator.cpp
  src/swath_generator.cpp
  src/route_generator.cpp
  src/path_generator.cpp
  src/visualizer.cpp
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
  ${FIELDS2COVER_LIBRARIES}
  GDAL::GDAL
)

# Fields2Cover headers as SYSTEM so their warnings don't fail our -Werror build.
if(Fields2Cover_INCLUDE_DIR)
  target_include_directories(${library_name} SYSTEM PUBLIC ${Fields2Cover_INCLUDE_DIR})
endif()

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "opennav_coverage::CoverageServer")

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

install(FILES test/test_field.xml
              test/cartesian_test_field.xml
              test/irregular_test_field.xml
  DESTINATION share/${PROJECT_NAME}/
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
