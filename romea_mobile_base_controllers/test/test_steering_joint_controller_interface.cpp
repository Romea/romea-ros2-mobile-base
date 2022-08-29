////gtest
//#include <gtest/gtest.h>

////ros
//#include <rclcpp/node.hpp>

////romea
//#include "romea_mobile_base_controllers/interfaces/steering_joint_controller_interface.hpp"

//class TestSteeringJointControllerInterface : public ::testing::Test
//{

//public:
//  TestSteeringJointControllerInterface():
//    state_value(0),
//    command_value(0),
//    state_hardware_interface("joint1",hardware_interface::HW_IF_POSITION,&state_value),
//    command_hardware_interface("joint1",hardware_interface::HW_IF_POSITION,&command_value),
//    state_loaned_interface(state_hardware_interface),
//    command_loaned_interface(command_hardware_interface)
//  {

//  }

//protected:
//  static void SetUpTestCase()
//  {
//    rclcpp::init(0, nullptr);
//  }

//  static void TearDownTestCase()
//  {
//    rclcpp::shutdown();
//  }

//  double state_value;
//  double command_value;
//  hardware_interface::StateInterface state_hardware_interface;
//  hardware_interface::CommandInterface command_hardware_interface;
//  hardware_interface::LoanedStateInterface state_loaned_interface;
//  hardware_interface::LoanedCommandInterface command_loaned_interface;
//  romea::SteeringJointControllerInterface joint_interface;

//};

//TEST_F(TestSteeringJointControllerInterface, checkHardwareInterFaceName)
//{
//  EXPECT_STREQ(romea::SteeringJointControllerInterface::hardware_interface_name("joint1").c_str(),"joint1/position");
//}

//TEST_F(TestSteeringJointControllerInterface, checkRead)
//{
//  state_value=4;
//  double measure;
//  joint_interface.read(state_loaned_interface,measure);
//  EXPECT_DOUBLE_EQ(measure,state_value);
//}

//TEST_F(TestSteeringJointControllerInterface, checkWrite)
//{
//  double command = 2;
//  joint_interface.write(command,command_loaned_interface);
//  EXPECT_DOUBLE_EQ(command_value,command);
//}
