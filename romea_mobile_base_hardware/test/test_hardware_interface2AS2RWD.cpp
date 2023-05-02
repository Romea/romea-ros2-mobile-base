// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



// std
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"

// romea
#include "../test/test_helper.h"
#include "../test/test_utils.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS2RWD.hpp"

class TestHarwareInterface2AS2RWD : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2AS2RWD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2AS2RWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
  }

  void MakeInterface(const std::string & command_interface_type)
  {
    interface = std::make_unique<romea::HardwareInterface2AS2RWD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2AS2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2AS2RWD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[1], "robot_joint2/position");
  check_interface_name(state_interfaces[2], "robot_joint3/position");
  check_interface_name(state_interfaces[5], "robot_joint4/position");
  check_interface_name(state_interfaces[8], "robot_joint5/position");
  check_interface_name(state_interfaces[9], "robot_joint6/position");
  check_interface_name(state_interfaces[10], "robot_joint7/position");
  check_interface_name(state_interfaces[11], "robot_joint8/position");
  check_interface_name(state_interfaces[12], "robot_joint9/position");
  check_interface_name(state_interfaces[15], "robot_joint10/position");
}

TEST_F(TestHarwareInterface2AS2RWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/position");
  check_interface_name(command_interfaces[2], "robot_joint3/velocity");
  check_interface_name(command_interfaces[3], "robot_joint4/velocity");
}

TEST_F(TestHarwareInterface2AS2RWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/position");
  check_interface_name(command_interfaces[2], "robot_joint3/effort");
  check_interface_name(command_interfaces[3], "robot_joint4/effort");
}


TEST_F(TestHarwareInterface2AS2RWD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2AS2RWD current_state;
  current_state.frontAxleSteeringAngle = 1.0;
  current_state.rearAxleSteeringAngle = 2.0;
  current_state.rearLeftWheelSpinningMotion.position = 3.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 4.0;
  current_state.rearLeftWheelSpinningMotion.torque = 5.0;
  current_state.rearRightWheelSpinningMotion.position = 6.0;
  current_state.rearRightWheelSpinningMotion.velocity = 7.0;
  current_state.rearRightWheelSpinningMotion.torque = 8.0;
  romea::SteeringAngleState front_left_wheel_steering_angle = 9.;
  romea::SteeringAngleState front_right_wheel_steering_angle = 10.;
  romea::SteeringAngleState rear_left_wheel_steering_angle = 11.;
  romea::SteeringAngleState rear_right_wheel_steering_angle = 12.;
  romea::RotationalMotionState front_left_wheel_spin_motion;
  front_left_wheel_spin_motion.position = 13.0;
  front_left_wheel_spin_motion.velocity = 14.0;
  front_left_wheel_spin_motion.torque = 15.0;
  romea::RotationalMotionState front_right_wheel_spin_motion;
  front_right_wheel_spin_motion.position = 16.0;
  front_right_wheel_spin_motion.velocity = 17.0;
  front_right_wheel_spin_motion.torque = 18.0;

  interface->set_state(
    current_state,
    front_left_wheel_steering_angle,
    front_right_wheel_steering_angle,
    rear_left_wheel_steering_angle,
    rear_right_wheel_steering_angle,
    front_left_wheel_spin_motion,
    front_right_wheel_spin_motion);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 18; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2AS2RWD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand2AS2RWD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rearAxleSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 4.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
