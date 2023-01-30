cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_controllers)

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
message(STATUS, "ros distro equal to  ${ROS_DISTRO}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/interfaces/controller_interface1FAS2FWD.cpp
  src/interfaces/controller_interface1FAS2RWD.cpp
  src/interfaces/controller_interface2AS4WD.cpp
  src/interfaces/controller_interface2FWS2FWD.cpp
  src/interfaces/controller_interface2FWS2RWD.cpp
  src/interfaces/controller_interface2FWS4WD.cpp
  src/interfaces/controller_interface2TD.cpp
  src/interfaces/controller_interface2WD.cpp
  src/interfaces/controller_interface4WD.cpp
  src/interfaces/controller_interface4WS4WD.cpp
  src/interfaces/controller_interface_common.cpp
  src/dead_reckoning.cpp
  src/dead_reckoning_publisher.cpp
  src/mobile_base_controller.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  romea_core_common
  romea_core_mobile_base
  romea_common_utils
  romea_mobile_base_msgs
  romea_mobile_base_utils
  controller_interface)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PUBLIC ROS_DISTRO=${ROS_DISTRO})

ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_mobile_base)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(romea_mobile_msgs)
ament_export_dependencies(romea_mobile_base_utils)
ament_export_dependencies(controller_interface)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

ament_package()



