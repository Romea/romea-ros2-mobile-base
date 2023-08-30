cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(four_wheel_steering_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_cmd_mux_msgs REQUIRED)
find_package(romea_cmd_mux_utils REQUIRED)


add_library(${PROJECT_NAME} SHARED
    src/control/command_interface.cpp
    src/conversions/command_conversions.cpp
    src/conversions/kinematic_conversions.cpp)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    romea_core_common
    romea_core_mobile_base
    romea_common_utils
    romea_mobile_base_msgs
    geometry_msgs
    four_wheel_steering_msgs
    ackermann_msgs
    romea_cmd_mux_msgs
    romea_cmd_mux_utils)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_mobile_base)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(four_wheel_steering_msgs)
ament_export_dependencies(ackermann_msgs)
ament_export_dependencies(romea_mobile_base_msgs)
ament_export_dependencies(romea_cmd_mux_msgs)
ament_export_dependencies(romea_cmd_mux_utils)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()
