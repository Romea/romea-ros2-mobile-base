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


TEST_F(TestHarwareInterface2FWS2FWD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(),"robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(),"robot_joint2");
  EXPECT_STREQ(state_interfaces[2].get_name().c_str(),"robot_joint3");
  EXPECT_STREQ(state_interfaces[5].get_name().c_str(),"robot_joint4");
  EXPECT_STREQ(state_interfaces[8].get_name().c_str(),"robot_joint5");
  EXPECT_STREQ(state_interfaces[11].get_name().c_str(),"robot_joint6");
}

TEST_F(TestHarwareInterface2FWS2FWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/velocity");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(),"robot_joint4/velocity");
}

TEST_F(TestHarwareInterface2FWS2FWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/effort");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(),"robot_joint4/effort");
}

TEST_F(TestHarwareInterface2FWS2FWD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2FWS2FWD current_state;
  current_state.frontLeftWheelSteeringAngle = 1.0;
  current_state.frontRightWheelSteeringAngle = 2.0;
  current_state.frontLeftWheelSpinningMotion.position=3.0;
  current_state.frontLeftWheelSpinningMotion.velocity=4.0;
  current_state.frontLeftWheelSpinningMotion.torque=5.0;
  current_state.frontRightWheelSpinningMotion.position=6.0;
  current_state.frontRightWheelSpinningMotion.velocity=7.0;
  current_state.frontRightWheelSpinningMotion.torque=8.0;
  romea::RotationalMotionState rear_left_wheel_spinning_set_point;
  rear_left_wheel_spinning_set_point.position = 9.0;
  rear_left_wheel_spinning_set_point.velocity = 10.0;
  rear_left_wheel_spinning_set_point.torque = 11.0;
  romea::RotationalMotionState rear_right_wheel_spinning_set_point;
  rear_right_wheel_spinning_set_point.position = 12.0;
  rear_right_wheel_spinning_set_point.velocity = 13.0;
  rear_right_wheel_spinning_set_point.torque = 14.0;


  interface->set_state(current_state,
                       rear_left_wheel_spinning_set_point,
                       rear_right_wheel_spinning_set_point);

  auto state_interfaces = interface->export_state_interfaces();
  for(size_t i=0;i<14;++i)
  {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(),i+1.0);
  }
}

TEST_F(TestHarwareInterface2FWS2FWD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for(size_t i=0;i<4;++i)
  {
    command_interfaces[i].set_value(i+1.0);
  }

  romea::HardwareCommand2FWS2FWD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint,3.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint,4.0);
}


