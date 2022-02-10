cmake_minimum_required(VERSION 3.5)
project(romea_mobile_base_msgs)

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
