find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_test_controller_interface1FAS2FWD test_controller_interface1FAS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface1FAS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface1FAS2RWD test_controller_interface1FAS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface1FAS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2AS4WD test_controller_interface2AS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2AS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2FWS2FWD test_controller_interface2FWS2FWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2FWS2FWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2FWS2RWD test_controller_interface2FWS2RWD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2FWS2RWD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2FWS4WD test_controller_interface2FWS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2FWS4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2TD test_controller_interface2TD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2TD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface2WD test_controller_interface2WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface2WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface4WD test_controller_interface4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface4WD ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_controller_interface4WS4WD test_controller_interface4WS4WD.cpp)
target_link_libraries(${PROJECT_NAME}_test_controller_interface4WS4WD ${PROJECT_NAME})
