find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_hardware_interface1FAS2FWD test_hardware_interface1FAS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_interface1FAS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_interface2AS4WD test_hardware_interface2AS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_interface2AS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_interface4WS4WD test_hardware_interface4WS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_interface4WS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_interface4WD test_hardware_interface4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_interface4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info test_hardware_info.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_handle test_hardware_handle.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_handle ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_spinning_joint_hardware_interface test_spinning_joint_hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME}_test_spinning_joint_hardware_interface ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_steering_joint_hardware_interface test_steering_joint_hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME}_test_steering_joint_hardware_interface ${PROJECT_NAME})
