find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_command_limits_parameters test_command_limits_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_command_limits_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_control_parameters test_control_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_control_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_iniertia_parameters test_inertia_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_iniertia_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_geometry_parameters test_geometry_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_geometry_parameters ${PROJECT_NAME})
