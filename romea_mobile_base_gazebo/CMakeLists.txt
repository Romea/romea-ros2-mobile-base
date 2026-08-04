cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_gazebo)

# if($ENV{ROS_DISTRO})
set(ROS_DISTRO $ENV{ROS_DISTRO})
# endif()

if(${ROS_DISTRO} STRGREATER_EQUAL "jazzy")

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

  set(PLUGINS_LIB_NAME "romea_mobile_base_gazebo")


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
  find_package(hardware_interface REQUIRED)
  find_package(pal_statistics REQUIRED)
  find_package(gz_ros2_control REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim REQUIRED)
  # find_package(gz_plugin_vendor REQUIRED)
  # find_package(gz-plugin REQUIRED)

  add_library(${PLUGINS_LIB_NAME} SHARED
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
    src/generic_gazebo_interface.cpp
    src/gazebo_system_interface.cpp
    src/gazebo_system_interface_legacy.cpp)

  ament_target_dependencies(${PLUGINS_LIB_NAME}
    rclcpp
    romea_core_common
    romea_core_mobile_base
    romea_common_utils
    romea_mobile_base_utils
    romea_mobile_base_hardware
    romea_mobile_base_simulation
    gz_ros2_control
    hardware_interface
    pal_statistics
    rclcpp_lifecycle)

  target_link_libraries(${PLUGINS_LIB_NAME}
    gz-sim::gz-sim
    #  gz-plugin::register
  )

  target_include_directories(${PLUGINS_LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  # target_compile_definitions(${PLUGINS_LIB_NAME} PUBLIC ROS_DISTRO=${ROS_DISTRO})

  ament_export_dependencies(rclcpp)
  ament_export_dependencies(rclcpp_lifecycle)
  ament_export_dependencies(romea_core_common)
  ament_export_dependencies(romea_core_mobile_base)
  ament_export_dependencies(romea_common_utils)
  ament_export_dependencies(romea_mobile_base_utils)
  ament_export_dependencies(romea_mobile_base_hardware)
  ament_export_dependencies(gz_ros2_control)
  ament_export_dependencies(hardware_interface)

  ament_export_include_directories(include)
  ament_export_libraries(${PLUGINS_LIB_NAME})
  ament_export_targets(export_${PLUGINS_LIB_NAME})

  install(TARGETS
    ${PLUGINS_LIB_NAME}
    DESTINATION lib/${PLUGINS_LIB_NAME})

  install(
    DIRECTORY include/
    DESTINATION include)

  install(
    TARGETS ${PLUGINS_LIB_NAME}
    EXPORT export_${PLUGINS_LIB_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test_legacy)
    add_subdirectory(test)
  endif()

  # pluginlib_export_plugin_description_file(gz_ros2_control plugins.xml)
  pluginlib_export_plugin_description_file(gz_ros2_control plugins.xml)
  ament_package()

endif()
