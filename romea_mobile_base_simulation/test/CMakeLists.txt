find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface1FAS2FWD test_simulation_interface1FAS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface1FAS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface1FAS2RWD test_simulation_interface1FAS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface1FAS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2AS4WD test_simulation_interface2AS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2AS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2FWS2FWD test_simulation_interface2FWS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2FWS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2FWS2RWD test_simulation_interface2FWS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2FWS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2TD test_simulation_interface2TD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2TD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2THD test_simulation_interface2THD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2THD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_simulation_interface2TTD test_simulation_interface2TTD.cpp)
target_link_libraries(${PROJECT_NAME}_test_simulation_interface2TTD ${PROJECT_NAME})

#ament_add_gtest(${PROJECT_NAME}_test_hardware_interface2FWS4WD test_hardware_interface2FWS4WD.cpp)
#target_link_libraries(${PROJECT_NAME}_test_hardware_interface2FWS4WD ${PROJECT_NAME})
