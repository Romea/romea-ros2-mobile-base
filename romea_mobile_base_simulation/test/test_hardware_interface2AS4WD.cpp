//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface2AS4WD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface2AS4WD.xacro";
    std::string urdf_file =  "/tmp/test_hardware_interface2AS4WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2AS4WD>(info[0],command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2AS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2AS4WD, checkStateInterfaceNames)
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
  EXPECT_STREQ(state_interfaces[24].get_name().c_str(),"robot_joint9");
  EXPECT_STREQ(state_interfaces[27].get_name().c_str(),"robot_joint10");

  //  EXPECT_STREQ(interface->front_axle_steering_joint.command.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->rear_axle_steering_joint.command.get_joint_name().c_str(),"robot_joint2");
  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint7");
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint8");
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint9");
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint10");


  //  EXPECT_STREQ(interface->front_axle_steering_joint.feedback.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->rear_axle_steering_joint.feedback.get_joint_name().c_str(),"robot_joint2");
  //  EXPECT_STREQ(interface->front_left_wheel_steering_joint_feedback.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->front_right_wheel_steering_joint_feedback.get_joint_name().c_str(),"robot_joint4");
  //  EXPECT_STREQ(interface->rear_left_wheel_steering_joint_feedback.get_joint_name().c_str(),"robot_joint5");
  //  EXPECT_STREQ(interface->rear_right_wheel_steering_joint_feedback.get_joint_name().c_str(),"robot_joint6");
  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint7");
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint8");
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint9");
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint10");
}

TEST_F(TestHarwareInterface2AS4WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/velocity");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(),"robot_joint4/velocity");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(),"robot_joint5/velocity");
  EXPECT_STREQ(command_interfaces[5].get_full_name().c_str(),"robot_joint6/velocity");

  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface2AS4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/effort");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(),"robot_joint4/effort");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(),"robot_joint5/effort");
  EXPECT_STREQ(command_interfaces[5].get_full_name().c_str(),"robot_joint6/effort");

  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}


TEST_F(TestHarwareInterface2AS4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2AS4WD current_state;
  current_state.frontAxleSteeringAngle = 1.0;
  current_state.rearAxleSteeringAngle = 2.0;
  current_state.frontLeftWheelSpinMotion.position=3.0;
  current_state.frontLeftWheelSpinMotion.velocity=4.0;
  current_state.frontLeftWheelSpinMotion.torque=5.0;
  current_state.frontRightWheelSpinMotion.position=6.0;
  current_state.frontRightWheelSpinMotion.velocity=7.0;
  current_state.frontRightWheelSpinMotion.torque=8.0;
  current_state.rearLeftWheelSpinMotion.position=9.0;
  current_state.rearLeftWheelSpinMotion.velocity=10.0;
  current_state.rearLeftWheelSpinMotion.torque=11.0;
  current_state.rearRightWheelSpinMotion.position=12.0;
  current_state.rearRightWheelSpinMotion.velocity=13.0;
  current_state.rearRightWheelSpinMotion.torque=14.0;
  romea::SteeringAngle front_left_wheel_steering_angle =15.;
  romea::SteeringAngle front_right_wheel_steering_angle = 16.;
  romea::SteeringAngle rear_left_wheel_steering_angle = 17.;
  romea::SteeringAngle rear_right_wheel_steering_angle = 18.;


  interface->set_state(current_state,
                       front_left_wheel_steering_angle,
                       front_right_wheel_steering_angle,
                       rear_left_wheel_steering_angle,
                       rear_right_wheel_steering_angle);

  auto state_interfaces = interface->export_state_interfaces();
  for(size_t i=0;i<18;++i)
  {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(),i+1.0);
  }
}

TEST_F(TestHarwareInterface2AS4WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for(size_t i=0;i<6;++i)
  {
    command_interfaces[i].set_value(i+1.0);
  }

  romea::HardwareCommand2AS4WD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rearAxleSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSetPoint,3.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSetPoint,4.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSetPoint,5.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSetPoint,6.0);

}

