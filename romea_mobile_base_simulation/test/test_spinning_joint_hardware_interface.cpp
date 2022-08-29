//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"
#include <hardware_interface/component_parser.hpp>

class TestSpinningJointHardwateInterface : public ::testing::Test
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
    joint_info.name= "spinning_wheel";
    joint_info.type= "joint";
    joint_info.command_interfaces.push_back({hardware_interface::HW_IF_POSITION,"-1","1","",0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_POSITION,"","","",0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_VELOCITY,"","","",0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_EFFORT,"","","",0});
  }

  void MakeJoint(const std::string & command_interface_type)
  {
    joint = std::make_unique<romea::SpinningJointHardwareInterface>(joint_info,command_interface_type);
  }

  std::unique_ptr<romea::SpinningJointHardwareInterface> joint;
  hardware_interface::ComponentInfo joint_info;
};

//TEST_F(TestSpinningJointHardwateInterface, checkInterfaceNames)
//{
//  MakeJoint(hardware_interface::HW_IF_VELOCITY);
//  EXPECT_STREQ(joint->command.get_joint_name().c_str(),"spinning_wheel");
//  EXPECT_STREQ(joint->feedback.position.get_joint_name().c_str(),"spinning_wheel");
//  EXPECT_STREQ(joint->feedback.velocity.get_joint_name().c_str(),"spinning_wheel");
//  EXPECT_STREQ(joint->feedback.torque.get_joint_name().c_str(),"spinning_wheel");
//}

//TEST_F(TestSpinningJointHardwateInterface, testInterfaceTypeWhenVelocityControlIsUsed)
//{
//  MakeJoint(hardware_interface::HW_IF_VELOCITY);
//  EXPECT_STREQ(joint->command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
//  EXPECT_STREQ(joint->feedback.position.get_interface_type().c_str(),hardware_interface::HW_IF_POSITION);
//  EXPECT_STREQ(joint->feedback.velocity.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
//  EXPECT_STREQ(joint->feedback.torque.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
//}

//TEST_F(TestSpinningJointHardwateInterface, testInterfaceTypeWhenEffortControlIsUsed)
//{
//  MakeJoint(hardware_interface::HW_IF_EFFORT);
//  EXPECT_STREQ(joint->command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
//  EXPECT_STREQ(joint->feedback.position.get_interface_type().c_str(),hardware_interface::HW_IF_POSITION);
//  EXPECT_STREQ(joint->feedback.velocity.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
//  EXPECT_STREQ(joint->feedback.torque.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
//}

TEST_F(TestSpinningJointHardwateInterface, checkExportedStateInterfaces)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  joint->export_state_interfaces(state_interfaces);

  EXPECT_EQ(state_interfaces.size(),3);
  EXPECT_STREQ(state_interfaces[0].get_full_name().c_str(),"spinning_wheel/position");
  EXPECT_STREQ(state_interfaces[1].get_full_name().c_str(),"spinning_wheel/velocity");
  EXPECT_STREQ(state_interfaces[2].get_full_name().c_str(),"spinning_wheel/effort");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenVelocityControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(),1);
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"spinning_wheel/velocity");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenEffortControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_EFFORT);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(),1);
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"spinning_wheel/effort");
}
