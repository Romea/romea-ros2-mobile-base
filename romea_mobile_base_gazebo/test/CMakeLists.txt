find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

function(add_gazebo_system_test test_name)
  ament_add_gtest(${PROJECT_NAME}_${test_name} ${test_name}.cpp)
  target_link_libraries(${PROJECT_NAME}_${test_name} ${PLUGINS_LIB_NAME})
  set_tests_properties(
    ${PROJECT_NAME}_${test_name}
    PROPERTIES ENVIRONMENT "ROS_HOME=/tmp/romea_mobile_base_gazebo_test")
endfunction()

add_gazebo_system_test(test_gazebo_system_interface)
