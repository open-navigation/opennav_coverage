cmake_minimum_required(VERSION 3.5)
project(opennav_row_coverage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(opennav_coverage REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)
set(executable_name opennav_row_coverage)
set(library_name ${executable_name}_core)

set(dependencies
  nav2_util
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

add_library(${library_name} SHARED
  src/row_swath_generator.cpp
  src/row_coverage_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name} Fields2Cover tinyxml2::tinyxml2)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} Fields2Cover tinyxml2::tinyxml2)

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
