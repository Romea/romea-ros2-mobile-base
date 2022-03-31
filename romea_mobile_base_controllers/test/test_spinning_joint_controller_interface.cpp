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
    radius(0.5)

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

  void MakeInterface(const std::string & joint_name)
  {
     joint_interface = std::make_unique<romea::SpinningJointControllerInterface>(joint_name,radius);
  }

  double state_value;
  double command_value;
  hardware_interface::StateInterface state_hardware_interface;
  hardware_interface::CommandInterface command_hardware_interface;
  hardware_interface::LoanedStateInterface state_loaned_interface;
  hardware_interface::LoanedCommandInterface command_loaned_interface;

  double radius;
  std::unique_ptr<romea::SpinningJointControllerInterface> joint_interface;

};

TEST_F(TestSpinningJointControllerInterface, checkInterFaceNames)
{
  MakeInterface("joint1");
  EXPECT_STREQ(joint_interface->get_state_interface_name().c_str(),"joint1/velocity");
  EXPECT_STREQ(joint_interface->get_command_interface_name().c_str(),"joint1/velocity");
}

TEST_F(TestSpinningJointControllerInterface, checkRegisterCommandInterface)
{
  MakeInterface("joint1");
  joint_interface->register_command_interface(command_loaned_interface);
}

TEST_F(TestSpinningJointControllerInterface, checkFailedToRegisterCommandInterface)
{
  MakeInterface("joint2");
  EXPECT_THROW(joint_interface->register_command_interface(command_loaned_interface),std::runtime_error);
}

TEST_F(TestSpinningJointControllerInterface, checkRegisterStateInterface)
{
  MakeInterface("joint1");
  joint_interface->register_state_interface(state_loaned_interface);
}

TEST_F(TestSpinningJointControllerInterface, checkFailedToRegisterStateInterface)
{
  MakeInterface("joint2");
  EXPECT_THROW(joint_interface->register_state_interface(state_loaned_interface),std::runtime_error);
}

TEST_F(TestSpinningJointControllerInterface, checkSetCommand)
{
  MakeInterface("joint1");
  joint_interface->register_command_interface(command_loaned_interface);
  joint_interface->set_command(2);
  EXPECT_DOUBLE_EQ(command_value,4);
}

TEST_F(TestSpinningJointControllerInterface, checkGetMeasurement)
{
  MakeInterface("joint1");
  joint_interface->register_state_interface(state_loaned_interface);
  state_value=4;
  EXPECT_DOUBLE_EQ(joint_interface->get_measurement(),2);
}
