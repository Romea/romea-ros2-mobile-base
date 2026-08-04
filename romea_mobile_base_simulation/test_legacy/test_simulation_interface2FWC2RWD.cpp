// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include <sstream>
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/node.hpp"

// romea
#include "test_helper.h"  // NOLINT
#include "romea_common_utils/joint_states.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWC2RWD.hpp"

class TestSimulationInterface2FWC2RWD : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2FWC2RWD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2FWC2RWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::ros2::SimulationInterface2FWC2RWD>(info[0], "velocity");
  }

  std::unique_ptr<romea::ros2::SimulationInterface2FWC2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};

TEST_F(TestSimulationInterface2FWC2RWD, checkGetCommand)
{
  auto command_interfaces = interface->export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 2u);
  command_interfaces[0].set_value(2.22829);
  command_interfaces[1].set_value(3.21196);

  auto simulation_command = interface->get_hardware_command();
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, 2.22829, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, 3.21196, 0.001);
}

TEST_F(TestSimulationInterface2FWC2RWD, checkGetCommandUsingJointState)
{
  auto command_interfaces = interface->export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 2u);
  command_interfaces[0].set_value(2.22829);
  command_interfaces[1].set_value(3.21196);

  auto simulation_command = interface->get_joint_state_command();
  ASSERT_EQ(simulation_command.name.size(), 2u);
  EXPECT_STREQ(simulation_command.name[0].c_str(), "robot_joint5");
  EXPECT_STREQ(simulation_command.name[1].c_str(), "robot_joint6");
  EXPECT_NEAR(simulation_command.velocity[0], 2.22829, 0.001);
  EXPECT_NEAR(simulation_command.velocity[1], 3.21196, 0.001);
}

TEST_F(TestSimulationInterface2FWC2RWD, checkGetState)
{
  romea::core::SimulationState2FWC2RWD simulation_state;
  simulation_state.frontLeftWheelSwivelingAngle = 1.0;
  simulation_state.frontRightWheelSwivelingAngle = 2.0;
  simulation_state.frontLeftWheelSpinningMotion.position = 3.0;
  simulation_state.frontLeftWheelSpinningMotion.velocity = 4.0;
  simulation_state.frontLeftWheelSpinningMotion.torque = 5.0;
  simulation_state.frontRightWheelSpinningMotion.position = 6.0;
  simulation_state.frontRightWheelSpinningMotion.velocity = 7.0;
  simulation_state.frontRightWheelSpinningMotion.torque = 8.0;
  simulation_state.rearLeftWheelSpinningMotion.position = 9.0;
  simulation_state.rearLeftWheelSpinningMotion.velocity = 10.0;
  simulation_state.rearLeftWheelSpinningMotion.torque = 11.0;
  simulation_state.rearRightWheelSpinningMotion.position = 12.0;
  simulation_state.rearRightWheelSpinningMotion.velocity = 13.0;
  simulation_state.rearRightWheelSpinningMotion.torque = 14.0;

  interface->set_feedback(simulation_state);

  auto state_interfaces = interface->export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 14u);
  for (size_t i = 0; i < state_interfaces.size(); ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestSimulationInterface2FWC2RWD, checkGetStateUsingJointState)
{
  auto feedback = romea::ros2::make_joint_state_msg(6);
  feedback.name[0] = "robot_joint1";
  feedback.name[1] = "robot_joint2";
  feedback.name[2] = "robot_joint3";
  feedback.name[3] = "robot_joint4";
  feedback.name[4] = "robot_joint5";
  feedback.name[5] = "robot_joint6";

  feedback.position[0] = 1.0;
  feedback.position[1] = 2.0;
  feedback.position[2] = 3.0;
  feedback.velocity[2] = 4.0;
  feedback.effort[2] = 5.0;
  feedback.position[3] = 6.0;
  feedback.velocity[3] = 7.0;
  feedback.effort[3] = 8.0;
  feedback.position[4] = 9.0;
  feedback.velocity[4] = 10.0;
  feedback.effort[4] = 11.0;
  feedback.position[5] = 12.0;
  feedback.velocity[5] = 13.0;
  feedback.effort[5] = 14.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 14u);
  for (size_t i = 0; i < state_interfaces.size(); ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
