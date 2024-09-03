find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_command_limits_parameters test_command_limits_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_command_limits_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_command_interface_parameters test_command_interface_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_command_interface_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_skid_steering_command_publisher test_skid_steering_command_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_test_skid_steering_command_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_omni_steering_command_publisher test_omni_steering_command_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_test_omni_steering_command_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_one_axle_steering_command_publisher test_one_axle_steering_command_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_test_one_axle_steering_command_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_two_axle_steering_command_publisher test_two_axle_steering_command_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_test_two_axle_steering_command_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_control_parameters test_mobile_base_control_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_control_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_iniertia_parameters test_mobile_base_inertia_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_iniertia_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_geometry_parameters test_mobile_base_geometry_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_geometry_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_1FAS2FWD test_mobile_base_parameters_1FAS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_1FAS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_1FAS2RWD test_mobile_base_parameters_1FAS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_1FAS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2AS4WD test_mobile_base_parameters_2AS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2AS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2FWS2FWD test_mobile_base_parameters_2FWS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2FWS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2FWS2RWD test_mobile_base_parameters_2FWS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2FWS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2FWS4WD test_mobile_base_parameters_2FWS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2FWS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_4WS4WD test_mobile_base_parameters_4WS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_4WS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_4WD test_mobile_base_parameters_4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2WD test_mobile_base_parameters_2WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2TD test_mobile_base_parameters_2TD.cpp)
target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2TD ${PROJECT_NAME})

#ament_add_gtest(${PROJECT_NAME}_test_mobile_base_parameters_2THD test_mobile_base_parameters_2THD.cpp)
#target_link_libraries(${PROJECT_NAME}_test_mobile_base_parameters_2THD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info1FASxxx test_hardware_info1FASxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info1FASxxx ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info2ASxxx test_hardware_info2ASxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info2ASxxx ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info2FWSxxx test_hardware_info2FWSxxx.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info2FWSxxx ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info2TD test_hardware_info2TD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info2TD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info2THD test_hardware_info2THD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info2THD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info2TTD test_hardware_info2TTD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info2TTD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info4WD test_hardware_info4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_info4WS4WD test_hardware_info4WS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_info4WS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_hardware_handle test_hardware_handle.cpp)
target_link_libraries(${PROJECT_NAME}_test_hardware_handle ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_spinning_joint_hardware_interface test_spinning_joint_hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME}_test_spinning_joint_hardware_interface ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_steering_joint_hardware_interface test_steering_joint_hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME}_test_steering_joint_hardware_interface ${PROJECT_NAME})