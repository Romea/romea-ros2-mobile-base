find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#ament_add_gtest(${PROJECT_NAME}_test_hardware_interface1FAS2FWD test_hardware_interface1FAS2FWD.cpp)
#target_link_libraries(${PROJECT_NAME}_test_hardware_interface1FAS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface4WD test_gazebo_interface4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface4WD ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface2FWSxxx test_gazebo_interface2FWSxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface2FWSxxx  ${PROJECT_NAME} libgazebo_test_fixture.a)
