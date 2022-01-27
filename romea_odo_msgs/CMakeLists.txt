cmake_minimum_required(VERSION 3.5)
project(romea_odo_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/KinematicMeasure.msg
    msg/KinematicMeasureStamped.msg
    msg/OneAxleSteeringCommand.msg
    msg/OneAxleSteeringMeasure.msg
    msg/OneAxleSteeringMeasureStamped.msg
    msg/SkidSteeringCommand.msg
    msg/SkidSteeringMeasure.msg
    msg/SkidSteeringMeasureStamped.msg
    msg/OmniSteeringMeasure.msg
    msg/OmniSteeringMeasureStamped.msg
    msg/OmniSteeringCommand.msg
    msg/TwoAxleSteeringMeasure.msg
    msg/TwoAxleSteeringMeasureStamped.msg
    msg/TwoAxleSteeringCommand.msg
    DEPENDENCIES std_msgs
    )

ament_export_dependencies(rosidl_default_runtime)
ament_package()

#cmake_minimum_required(VERSION 2.8.3)
#project(romea_odo_msgs)

#set(${PROJECT_NAME}_CATKIN_DEPS std_msgs geometry_msgs message_generation)
#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

## Generate messages in the 'msg' folder
# add_message_files(
#  DIRECTORY
#  msg
#  FILES
#  KinematicMeasure.msg
#  KinematicMeasureStamped.msg
#  OneAxleSteeringCommand.msg
#  OneAxleSteeringMeasure.msg
#  OneAxleSteeringMeasureStamped.msg
#  SkidSteeringCommand.msg
#  SkidSteeringMeasure.msg
#  SkidSteeringMeasureStamped.msg
#  OmniSteeringMeasure.msg
#  OmniSteeringMeasureStamped.msg
#  OmniSteeringCommand.msg
#  TwoAxleSteeringMeasure.msg
#  TwoAxleSteeringMeasureStamped.msg
#  TwoAxleSteeringCommand.msg
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

## Generates cmake config files for your package
#catkin_package(
#   CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#)

