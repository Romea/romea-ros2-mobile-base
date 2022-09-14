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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(controller_interface REQUIRED)

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
 src/mobile_base_controller.cpp
 )

ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 romea_core_common
 romea_core_mobile_base
 romea_common_utils
 romea_mobile_base_msgs
 romea_mobile_base_utils
 controller_interface
 )

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)


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
 DESTINATION include
 )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

ament_package()




#cmake_minimum_required(VERSION 3.0.2)
#project(romea_mobile_base_controllers)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

########################
### Find dependencies ##
########################
#set(${PROJECT_NAME}_CATKIN_DEPS
# roscpp
# roslib
# controller_interface
# hardware_interface
# realtime_tools
# romea_mobile_base_utils
# romea_odo_msgs
# tf2_geometry_msgs
# sensor_msgs
# nav_msgs)

#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#find_package(romea_core_common REQUIRED)
#find_package(romea_mobile_base REQUIRED)

#find_package( PkgConfig )
#pkg_check_modules( EIGEN3 REQUIRED eigen3 )

####################################
### catkin specific configuration ##
####################################
#catkin_package(
# INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
# )

############
### build ##
############
#include_directories(
# include
# SYSTEM ${catkin_INCLUDE_DIRS}
# SYSTEM ${romea_core_common_INCLUDE_DIRS}
# SYSTEM ${romea_mobile_base_INCLUDE_DIRS}
# SYSTEM ${EIGEN3_INCLUDE_DIRS}
# )

#set(romea_LIBRARIES
# ${romea_core_common_LIBRARIES}
# ${romea_mobile_base_LIBRARIES}
#)

#add_library(romea_odometry_frame_interfaces
# src/interfaces/joint_interface.cpp
# src/interfaces/spinning_joint_interface.cpp
# src/interfaces/steering_joint_interface.cpp
# src/interfaces/odometry_interface1FAS2FWD.cpp
# src/interfaces/odometry_interface1FAS2RWD.cpp
# src/interfaces/odometry_interface1FWS2RWD.cpp
# src/interfaces/odometry_interface2AS4WD.cpp
# src/interfaces/odometry_interface2FWS4WD.cpp
# src/interfaces/odometry_interface2FWS2RWD.cpp
# src/interfaces/odometry_interface2FWS2FWD.cpp
# src/interfaces/odometry_interface2WD.cpp
# src/interfaces/odometry_interface4WD.cpp
# src/interfaces/odometry_interface4WS4WD.cpp)

#add_library(romea_odometry_controller_plugins
# src/odometry_controller.cpp
# src/enhanced_odometry_controller.cpp
# src/dead_reckoning_publisher.cpp
# src/dead_reckoning.cpp
# )

#target_link_libraries(romea_odometry_controller_plugins
# ${catkin_LIBRARIES}
# ${romea_LIBRARIES}
# romea_odometry_frame_interfaces
# )
