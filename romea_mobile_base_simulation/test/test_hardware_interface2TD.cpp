//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface2TD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface2TD.xacro";
    std::string urdf_file =  "/tmp/test_hardware_interface2TD.urdf";
    std::string cmd = "xacro "+xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
  }

  void MakeInterface(const std::string & command_interface_type)
  {
    interface = std::make_unique<romea::HardwareInterface2TD>(info[0],command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2TD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2TD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(),"robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(),"robot_joint2");
  EXPECT_STREQ(state_interfaces[6].get_name().c_str(),"robot_joint3");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(),"robot_joint4");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->right_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint4");
}

TEST_F(TestHarwareInterface2TD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/velocity");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface2TD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/effort");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}

TEST_F(TestHarwareInterface2TD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2TD current_state;
  current_state.leftSprocketWheelSpinMotion.position=1.0;
  current_state.leftSprocketWheelSpinMotion.velocity=2.0;
  current_state.leftSprocketWheelSpinMotion.torque=3.0;
  current_state.rightSprocketWheelSpinMotion.position=4.0;
  current_state.rightSprocketWheelSpinMotion.velocity=5.0;
  current_state.rightSprocketWheelSpinMotion.torque=6.0;
  romea::RotationalMotionState leftIdlerWheelSpinMotion;
  leftIdlerWheelSpinMotion.position=7.0;
  leftIdlerWheelSpinMotion.velocity=8.0;
  leftIdlerWheelSpinMotion.torque=9.0;
  romea::RotationalMotionState rightIdlerWheelSpinMotion;
  rightIdlerWheelSpinMotion.position=10.0;
  rightIdlerWheelSpinMotion.velocity=11.0;
  rightIdlerWheelSpinMotion.torque=12.0;

  interface->set_state(current_state,
                       leftIdlerWheelSpinMotion,
                       rightIdlerWheelSpinMotion);

  auto state_interfaces = interface->export_state_interfaces();
  for(size_t i=0;i<12;++i)
  {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(),i+1.0);
  }
}

TEST_F(TestHarwareInterface2TD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for(size_t i=0;i<2;++i)
  {
    command_interfaces[i].set_value(i+1.0);
  }

  romea::HardwareCommand2TD current_command = interface->get_command();
  EXPECT_DOUBLE_EQ(current_command.leftSprocketWheelSetPoint,1.0);
  EXPECT_DOUBLE_EQ(current_command.rightSprocketWheelSetPoint,2.0);

}


