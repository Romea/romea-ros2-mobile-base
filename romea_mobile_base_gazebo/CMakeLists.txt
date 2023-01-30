cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_gazebo)

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

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_mobile_base_hardware REQUIRED)
find_package(romea_mobile_base_simulation REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/spinning_joint_gazebo_interface.cpp
  src/steering_joint_gazebo_interface.cpp
  src/gazebo_interface1FASxxx.cpp
  src/gazebo_interface2ASxxx.cpp
  src/gazebo_interface2FWSxxx.cpp
  src/gazebo_interface2TD.cpp
  src/gazebo_interface2THD.cpp
  src/gazebo_interface2TTD.cpp
  src/gazebo_interface2WD.cpp
  src/gazebo_interface4WD.cpp
  src/gazebo_interface4WS4WD.cpp
  src/gazebo_system_interface.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  romea_core_common
  romea_core_mobile_base
  romea_common_utils
  romea_mobile_base_utils
  romea_mobile_base_hardware
  romea_mobile_base_simulation
  gazebo_dev
  gazebo_ros
  gazebo_ros2_control
  hardware_interface
  rclcpp_lifecycle)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PUBLIC ROS_DISTRO=${ROS_DISTRO})

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_mobile_base)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(romea_mobile_base_utils)
ament_export_dependencies(romea_mobile_base_hardware)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(gazebo_ros2_control)
ament_export_dependencies(hardware_interface)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(gazebo_ros2_control plugins.xml)

ament_package()
