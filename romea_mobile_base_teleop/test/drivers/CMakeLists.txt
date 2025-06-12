find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_skid_steering_teleop test_skid_steering_teleop.cpp)
target_link_libraries(${PROJECT_NAME}_test_skid_steering_teleop ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_omni_steering_teleop test_omni_steering_teleop.cpp)
target_link_libraries(${PROJECT_NAME}_test_omni_steering_teleop ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_one_axle_steering_teleop test_one_axle_steering_teleop.cpp)
target_link_libraries(${PROJECT_NAME}_test_one_axle_steering_teleop ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_two_axle_steering_teleop_2_axes test_two_axle_steering_teleop_2_axes.cpp)
target_link_libraries(${PROJECT_NAME}_test_two_axle_steering_teleop_2_axes ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_two_axle_steering_teleop_4_axes test_two_axle_steering_teleop_4_axes.cpp)
target_link_libraries(${PROJECT_NAME}_test_two_axle_steering_teleop_4_axes ${PROJECT_NAME})
