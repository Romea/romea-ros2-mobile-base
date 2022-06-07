//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface2FWS2FWD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface2FWS2FWD.xacro";
    std::string urdf_file =  "/tmp/test_hardware_interface2FWS2FWD.urdf";
    std::string cmd = "xacro "+xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
  }

  void MakeInterface(const std::string & command_interface_type)
  {
    interface = std::make_unique<romea::HardwareInterface2FWS2FWD>(info[0],command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2FWS2FWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2FWS2FWD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  EXPECT_STREQ(interface->front_left_wheel_steering_joint.feedback.get_joint_name().c_str(),"robot_joint1");
  EXPECT_STREQ(interface->front_right_wheel_steering_joint.feedback.get_joint_name().c_str(),"robot_joint2");
  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint3");
  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint4");
  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint5");
  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint6");
}

TEST_F(TestHarwareInterface2FWS2FWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface2FWS2FWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}


