//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_controllers/interfaces/spinning_joint_controller_interface.hpp"

class TestSpinningJointControllerInterface : public ::testing::Test
{

public:
  TestSpinningJointControllerInterface():
    state_value(0),
    command_value(1),
    state_hardware_interface("joint1",hardware_interface::HW_IF_VELOCITY,&state_value),
    command_hardware_interface("joint1",hardware_interface::HW_IF_VELOCITY,&command_value),
    state_loaned_interface(state_hardware_interface),
    command_loaned_interface(command_hardware_interface),
    joint_interface(0.5)

  {

  }
protected:

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }


  double state_value;
  double command_value;
  hardware_interface::StateInterface state_hardware_interface;
  hardware_interface::CommandInterface command_hardware_interface;
  hardware_interface::LoanedStateInterface state_loaned_interface;
  hardware_interface::LoanedCommandInterface command_loaned_interface;
  romea::SpinningJointControllerInterface joint_interface;

};

TEST_F(TestSpinningJointControllerInterface, checkHardwareInterFaceName)
{
  EXPECT_STREQ(romea::SpinningJointControllerInterface::hardware_interface_name("joint1").c_str(),"joint1/velocity");
}

TEST_F(TestSpinningJointControllerInterface, checkRead)
{
  state_value=4;
  double measure;
  joint_interface.read(state_loaned_interface,measure);
  EXPECT_DOUBLE_EQ(measure,2);
}

TEST_F(TestSpinningJointControllerInterface, checkWrite)
{
  double command = 2;
  joint_interface.write(command,command_loaned_interface);
  EXPECT_DOUBLE_EQ(command_value,4);
}
