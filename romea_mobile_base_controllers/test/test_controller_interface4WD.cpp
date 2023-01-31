// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_mobile_base_controllers/interfaces/controller_interface4WD.hpp"

class TestControllerInterface4WD : public ::testing::Test
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
    rclcpp::NodeOptions no;
    no.arguments(
      {"--ros-args",
        "-p", "joints.front_left_wheel_spinning_joint_name:=J1",
        "-p", "joints.front_right_wheel_spinning_joint_name:=J2",
        "-p", "joints.rear_left_wheel_spinning_joint_name:=J3",
        "-p", "joints.rear_right_wheel_spinning_joint_name:=J4"
      });

    node = std::make_shared<romea::HardwareInterfaceNode>("test_interface_controller_4WD", no);

    state_values.resize(4);
    command_values.resize(4);

    romea::ControllerInterface4WD::declare_joints_names(node, "joints");
    joints_names = romea::ControllerInterface4WD::get_joints_names(node, "joints");

    state_hardware_interfaces.emplace_back(
      joints_names[0], hardware_interface::HW_IF_VELOCITY,
      &state_values[0]);
    state_hardware_interfaces.emplace_back(
      joints_names[1], hardware_interface::HW_IF_VELOCITY,
      &state_values[1]);
    state_hardware_interfaces.emplace_back(
      joints_names[2], hardware_interface::HW_IF_VELOCITY,
      &state_values[2]);
    state_hardware_interfaces.emplace_back(
      joints_names[3], hardware_interface::HW_IF_VELOCITY,
      &state_values[3]);

    command_hardware_interfaces.emplace_back(
      joints_names[0], hardware_interface::HW_IF_VELOCITY,
      &command_values[0]);
    command_hardware_interfaces.emplace_back(
      joints_names[1], hardware_interface::HW_IF_VELOCITY,
      &command_values[1]);
    command_hardware_interfaces.emplace_back(
      joints_names[2], hardware_interface::HW_IF_VELOCITY,
      &command_values[2]);
    command_hardware_interfaces.emplace_back(
      joints_names[3], hardware_interface::HW_IF_VELOCITY,
      &command_values[3]);

    for (auto & state_hardware_interface : state_hardware_interfaces) {
      state_loaned_interfaces.emplace_back(state_hardware_interface);
    }

    for (auto & command_hardware_interface : command_hardware_interfaces) {
      command_loaned_interfaces.emplace_back(command_hardware_interface);
    }

    mobile_info.geometry.rearAxle.wheels.radius = 0.5;
    mobile_info.geometry.frontAxle.wheels.radius = 0.5;

    controller_interface = std::make_unique<romea::ControllerInterface4WD>(mobile_info);
  }

  std::shared_ptr<romea::HardwareInterfaceNode> node;

  std::vector<double> state_values;
  std::vector<double> command_values;
  std::vector<hardware_interface::StateInterface> state_hardware_interfaces;
  std::vector<hardware_interface::CommandInterface> command_hardware_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_loaned_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> command_loaned_interfaces;

  romea::MobileBaseInfo4WD mobile_info;
  std::vector<std::string> joints_names;
  std::unique_ptr<romea::ControllerInterface4WD> controller_interface;
};

TEST_F(TestControllerInterface4WD, checkHardwareInterfaceNames)
{
  auto hardware_interface_names = romea::ControllerInterface4WD::hardware_interface_names(
    joints_names);
  EXPECT_STREQ(hardware_interface_names[0].c_str(), "J1/velocity");
  EXPECT_STREQ(hardware_interface_names[1].c_str(), "J2/velocity");
  EXPECT_STREQ(hardware_interface_names[2].c_str(), "J3/velocity");
  EXPECT_STREQ(hardware_interface_names[3].c_str(), "J4/velocity");
}

TEST_F(TestControllerInterface4WD, checkWrite)
{
  romea::OdometryFrame4WD command;
  command.frontLeftWheelLinearSpeed = 1;
  command.frontRightWheelLinearSpeed = 2;
  command.rearLeftWheelLinearSpeed = 3;
  command.rearRightWheelLinearSpeed = 4;

  controller_interface->write(command, command_loaned_interfaces);
  EXPECT_EQ(command_values[0], 2);
  EXPECT_EQ(command_values[1], 4);
  EXPECT_EQ(command_values[2], 6);
  EXPECT_EQ(command_values[3], 8);
}

TEST_F(TestControllerInterface4WD, checkRead)
{
  state_values[0] = 2;
  state_values[1] = 4;
  state_values[2] = 6;
  state_values[3] = 8;

  romea::OdometryFrame4WD measure;
  controller_interface->read(state_loaned_interfaces, measure);
  EXPECT_EQ(measure.frontLeftWheelLinearSpeed, 1);
  EXPECT_EQ(measure.frontRightWheelLinearSpeed, 2);
  EXPECT_EQ(measure.rearLeftWheelLinearSpeed, 3);
  EXPECT_EQ(measure.rearRightWheelLinearSpeed, 4);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
