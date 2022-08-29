//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface1FWS2RWD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface1FWS2RWD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface1FWS2RWD.xacro";
    std::string urdf_file =  "/tmp/test_hardware_interface1FWS2RWD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface1FWS2RWD>(info[0],command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface1FWS2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface1FWS2RWD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(),"robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(),"robot_joint2");
  EXPECT_STREQ(state_interfaces[4].get_name().c_str(),"robot_joint3");
  EXPECT_STREQ(state_interfaces[7].get_name().c_str(),"robot_joint4");
}

TEST_F(TestHarwareInterface1FWS2RWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/velocity");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/velocity");
}

TEST_F(TestHarwareInterface1FWS2RWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(),"robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(),"robot_joint2/effort");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(),"robot_joint3/effort");
}

TEST_F(TestHarwareInterface1FWS2RWD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState1FWS2RWD current_state;
  current_state.frontWheelSteeringAngle = 1.0;
  current_state.rearLeftWheelSpinMotion.position=2.0;
  current_state.rearLeftWheelSpinMotion.velocity=3.0;
  current_state.rearLeftWheelSpinMotion.torque=4.0;
  current_state.rearRightWheelSpinMotion.position=5.0;
  current_state.rearRightWheelSpinMotion.velocity=6.0;
  current_state.rearRightWheelSpinMotion.torque=7.0;
  romea::RotationalMotionState front_wheel_set_point;
  front_wheel_set_point.position =8.0;
  front_wheel_set_point.velocity = 9.0;
  front_wheel_set_point.torque = 10.0;

  interface->set_state(current_state,front_wheel_set_point);

  auto state_interfaces = interface->export_state_interfaces();
  for(size_t i=0;i<10;++i)
  {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(),i+1.0);
  }
}

TEST_F(TestHarwareInterface1FWS2RWD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for(size_t i=0;i<3;++i)
  {
    command_interfaces[i].set_value(i+1.0);
  }

  romea::HardwareCommand1FWS2RWD current_command = interface->get_command();
  EXPECT_DOUBLE_EQ(current_command.frontWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSetPoint,2.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSetPoint,3.0);
}


