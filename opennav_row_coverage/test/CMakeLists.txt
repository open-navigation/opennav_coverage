# Each test links ${library_name}, which PUBLIC-links all dependency targets
# (see ../CMakeLists.txt), so the dependencies propagate transitively. Rolling
# removed ament_target_dependencies, hence the direct linking.

#Test Swath generation
ament_add_gtest(test_swath_generator
  test_swath_generator.cpp
)
target_link_libraries(test_swath_generator
  ${library_name}
)

#Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
target_link_libraries(test_utils
  ${library_name}
)
