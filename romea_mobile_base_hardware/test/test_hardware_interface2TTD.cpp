//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface2TTD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface2TTD.xacro";
    std::string urdf_file =  "/tmp/test_hardware_interface2TTD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2TTD>(info[0],command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2TTD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2TTD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(),"robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(),"robot_joint2");
  EXPECT_STREQ(state_interfaces[6].get_name().c_str(),"robot_joint3");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(),"robot_joint4");
  EXPECT_STREQ(state_interfaces[12].get_name().c_str(),"robot_joint5");
  EXPECT_STREQ(state_interfaces[15].get_name().c_str(),"robot_joint6");
  EXPECT_STREQ(state_interfaces[18].get_name().c_str(),"robot_joint7");
  EXPECT_STREQ(state_interfaces[21].get_name().c_str(),"robot_joint8");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->right_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint4");
  //  EXPECT_STREQ(interface->front_left_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint5");
  //  EXPECT_STREQ(interface->front_right_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint6");
  //  EXPECT_STREQ(interface->rear_left_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint7");
  //  EXPECT_STREQ(interface->rear_right_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint8");
}

TEST_F(TestHarwareInterface2TTD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/velocity");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface2TTD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/effort");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}

TEST_F(TestHarwareInterface2TTD, checkSetCurrentState)
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
  romea::RotationalMotionState frontLeftRollerWheelSpinMotion;
  frontLeftRollerWheelSpinMotion.position=13.0;
  frontLeftRollerWheelSpinMotion.velocity=14.0;
  frontLeftRollerWheelSpinMotion.torque=15.0;
  romea::RotationalMotionState frontRightRollerWheelSpinMotion;
  frontRightRollerWheelSpinMotion.position=16.0;
  frontRightRollerWheelSpinMotion.velocity=17.0;
  frontRightRollerWheelSpinMotion.torque=18.0;
  romea::RotationalMotionState rearLeftRollerWheelSpinMotion;
  rearLeftRollerWheelSpinMotion.position=19.0;
  rearLeftRollerWheelSpinMotion.velocity=20.0;
  rearLeftRollerWheelSpinMotion.torque=21.0;
  romea::RotationalMotionState rearRightRollerWheelSpinMotion;
  rearRightRollerWheelSpinMotion.position=22.0;
  rearRightRollerWheelSpinMotion.velocity=23.0;
  rearRightRollerWheelSpinMotion.torque=24.0;

  interface->set_state(current_state,
                       leftIdlerWheelSpinMotion,
                       rightIdlerWheelSpinMotion,
                       frontLeftRollerWheelSpinMotion,
                       frontRightRollerWheelSpinMotion,
                       rearLeftRollerWheelSpinMotion,
                       rearRightRollerWheelSpinMotion);

  auto state_interfaces = interface->export_state_interfaces();
  for(size_t i=0;i<24;++i)
  {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(),i+1.0);
  }
}

TEST_F(TestHarwareInterface2TTD, checkGetCurrentCommand)
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

