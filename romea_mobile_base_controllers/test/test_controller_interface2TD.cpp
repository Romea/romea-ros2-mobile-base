// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"

class TestControllerInterface2TD : public ::testing::Test
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
        "-p", "joints.left_sprocket_wheel_spinning_joint_name:=J1",
        "-p", "joints.right_sprocket_wheel_spinning_joint_name:=J2",
      });

    node = std::make_shared<romea::HardwareInterfaceNode>("test_interface_controller_2TD", no);

    state_values.resize(2);
    command_values.resize(2);

    romea::ControllerInterface2TD::declare_joints_names(node, "joints");
    joints_names = romea::ControllerInterface2TD::get_joints_names(node, "joints");

    state_hardware_interfaces.emplace_back(
      joints_names[0], hardware_interface::HW_IF_VELOCITY,
      &state_values[0]);
    state_hardware_interfaces.emplace_back(
      joints_names[1], hardware_interface::HW_IF_VELOCITY,
      &state_values[1]);

    command_hardware_interfaces.emplace_back(
      joints_names[0], hardware_interface::HW_IF_VELOCITY,
      &command_values[0]);
    command_hardware_interfaces.emplace_back(
      joints_names[1], hardware_interface::HW_IF_VELOCITY,
      &command_values[1]);

    for (auto & state_hardware_interface : state_hardware_interfaces) {
      state_loaned_interfaces.emplace_back(state_hardware_interface);
    }

    for (auto & command_hardware_interface : command_hardware_interfaces) {
      command_loaned_interfaces.emplace_back(command_hardware_interface);
    }

    mobile_info.geometry.tracks.thickness = 0.1;
    mobile_info.geometry.tracks.sprocketWheel.radius = 0.4;

    controller_interface = std::make_unique<romea::ControllerInterface2TD>(mobile_info);
  }

  std::shared_ptr<romea::HardwareInterfaceNode> node;

  std::vector<double> state_values;
  std::vector<double> command_values;
  std::vector<hardware_interface::StateInterface> state_hardware_interfaces;
  std::vector<hardware_interface::CommandInterface> command_hardware_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_loaned_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> command_loaned_interfaces;

  romea::MobileBaseInfo2TD mobile_info;
  std::vector<std::string> joints_names;
  std::unique_ptr<romea::ControllerInterface2TD> controller_interface;
};


TEST_F(TestControllerInterface2TD, checkHardwareInterfaceNames)
{
  auto hardware_interface_names = romea::ControllerInterface2TD::hardware_interface_names(
    joints_names);
  EXPECT_STREQ(hardware_interface_names[0].c_str(), "J1/velocity");
  EXPECT_STREQ(hardware_interface_names[1].c_str(), "J2/velocity");
}

TEST_F(TestControllerInterface2TD, checkWrite)
{
  romea::OdometryFrame2TD command;
  command.leftTrackLinearSpeed = 1;
  command.rightTrackLinearSpeed = 2;

  controller_interface->write(command, command_loaned_interfaces);
  EXPECT_EQ(command_values[0], 2);
  EXPECT_EQ(command_values[1], 4);
}

TEST_F(TestControllerInterface2TD, checkRead)
{
  state_values[0] = 2;
  state_values[1] = 4;

  romea::OdometryFrame2TD measure;
  controller_interface->read(state_loaned_interfaces, measure);
  EXPECT_EQ(measure.leftTrackLinearSpeed, 1);
  EXPECT_EQ(measure.rightTrackLinearSpeed, 2);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
