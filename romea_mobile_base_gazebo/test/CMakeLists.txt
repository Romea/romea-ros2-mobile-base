find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface1FASxxx test_gazebo_interface1FASxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface1FASxxx ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface4WD test_gazebo_interface4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface4WD ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface2FWSxxx test_gazebo_interface2FWSxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface2FWSxxx  ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface2ASxxx test_gazebo_interface2ASxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface2ASxxx  ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface2TD test_gazebo_interface2TD.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface2TD  ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface2THD test_gazebo_interface2THD.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface2THD  ${PROJECT_NAME} libgazebo_test_fixture.a)

ament_add_gtest(${PROJECT_NAME}_test_gazebo_interface4WS4WD test_gazebo_interface4WS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_gazebo_interface4WS4WD ${PROJECT_NAME} libgazebo_test_fixture.a)
