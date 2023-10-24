# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ${dependencies}
)
target_link_libraries(test_utils
  ${library_name}
)

# Test robot
ament_add_gtest(test_robot
  test_robot.cpp
)
ament_target_dependencies(test_robot
  ${dependencies}
)
target_link_libraries(test_robot
  ${library_name}
)

# Test visualizations
ament_add_gtest(test_viz
  test_viz.cpp
)
ament_target_dependencies(test_viz
  ${dependencies}
)
target_link_libraries(test_viz
  ${library_name}
)

# Test headland
ament_add_gtest(test_headland
  test_headland.cpp
)
ament_target_dependencies(test_headland
  ${dependencies}
)
target_link_libraries(test_headland
  ${library_name}
)

# Test swath
ament_add_gtest(test_swath
  test_swath.cpp
)
ament_target_dependencies(test_swath
  ${dependencies}
)
target_link_libraries(test_swath
  ${library_name}
)

# Test route
ament_add_gtest(test_route
  test_route.cpp
)
ament_target_dependencies(test_route
  ${dependencies}
)
target_link_libraries(test_route
  ${library_name}
)

# Test path
ament_add_gtest(test_path
  test_path.cpp
)
ament_target_dependencies(test_path
  ${dependencies}
)
target_link_libraries(test_path
  ${library_name}
)

# Test server
ament_add_gtest(test_server
  test_server.cpp
)
ament_target_dependencies(test_server
  ${dependencies}
)
target_link_libraries(test_server
  ${library_name}
)
