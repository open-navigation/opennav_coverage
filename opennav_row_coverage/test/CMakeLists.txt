#Test Swath generation
ament_add_gtest(test_swath_generator
  test_swath_generator.cpp
)
ament_target_dependencies(test_swath_generator
  ${dependencies}
)
target_link_libraries(test_swath_generator
  ${library_name}
)

#Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ${dependencies}
)
target_link_libraries(test_utils
  ${library_name}
)
