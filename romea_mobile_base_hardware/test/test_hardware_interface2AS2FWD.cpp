// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>
#include <hardware_interface/component_parser.hpp>

// std
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface2AS2FWD.hpp"

class TestHarwareInterface2AS2FWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2AS2FWD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2AS2FWD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2AS2FWD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2AS2FWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2AS2FWD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[2].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[5].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[8].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(), "robot_joint6");
  EXPECT_STREQ(state_interfaces[10].get_name().c_str(), "robot_joint7");
  EXPECT_STREQ(state_interfaces[11].get_name().c_str(), "robot_joint8");
  EXPECT_STREQ(state_interfaces[12].get_name().c_str(), "robot_joint9");
  EXPECT_STREQ(state_interfaces[15].get_name().c_str(), "robot_joint10");
}

TEST_F(TestHarwareInterface2AS2FWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/velocity");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/velocity");
}

TEST_F(TestHarwareInterface2AS2FWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/effort");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/effort");
}


TEST_F(TestHarwareInterface2AS2FWD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2AS2FWD current_state;
  current_state.frontAxleSteeringAngle = 1.0;
  current_state.rearAxleSteeringAngle = 2.0;
  current_state.frontLeftWheelSpinningMotion.position = 3.0;
  current_state.frontLeftWheelSpinningMotion.velocity = 4.0;
  current_state.frontLeftWheelSpinningMotion.torque = 5.0;
  current_state.frontRightWheelSpinningMotion.position = 6.0;
  current_state.frontRightWheelSpinningMotion.velocity = 7.0;
  current_state.frontRightWheelSpinningMotion.torque = 8.0;
  romea::SteeringAngleState front_left_wheel_steering_angle = 9.;
  romea::SteeringAngleState front_right_wheel_steering_angle = 10.;
  romea::SteeringAngleState rear_left_wheel_steering_angle = 11.;
  romea::SteeringAngleState rear_right_wheel_steering_angle = 12.;
  romea::RotationalMotionState rear_left_wheel_spin_motion;
  rear_left_wheel_spin_motion.position = 13.0;
  rear_left_wheel_spin_motion.velocity = 14.0;
  rear_left_wheel_spin_motion.torque = 15.0;
  romea::RotationalMotionState rear_right_wheel_spin_motion;
  rear_right_wheel_spin_motion.position = 16.0;
  rear_right_wheel_spin_motion.velocity = 17.0;
  rear_right_wheel_spin_motion.torque = 18.0;


  interface->set_state(
    current_state,
    front_left_wheel_steering_angle,
    front_right_wheel_steering_angle,
    rear_left_wheel_steering_angle,
    rear_right_wheel_steering_angle,
    rear_left_wheel_spin_motion,
    rear_right_wheel_spin_motion);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 18; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2AS2FWD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand2AS2FWD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rearAxleSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 4.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
