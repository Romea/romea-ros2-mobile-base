//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"

class TestControllerInterface2TD : public ::testing::Test
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

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args",
                  "-p","joints.left_sprocket_wheel_spinning_joint_name:=J1",
                  "-p","joints.right_sprocket_wheel_spinning_joint_name:=J2",
                 });

    node = std::make_shared<rclcpp::Node>("test_interface_controller_2TD", no);

    state_values.resize(2);
    command_values.resize(2);

    romea::ControllerInterface2TD::declare_joints_names(node,"joints");
    joints_names = romea::ControllerInterface2TD::get_joints_names(node,"joints");

    state_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_VELOCITY,&state_values[0]);
    state_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_VELOCITY,&state_values[1]);

    command_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_VELOCITY,&command_values[0]);
    command_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_VELOCITY,&command_values[1]);

    for(auto & state_hardware_interface : state_hardware_interfaces)
      state_loaned_interfaces.emplace_back(state_hardware_interface);

    for(auto & command_hardware_interface : command_hardware_interfaces)
      command_loaned_interfaces.emplace_back(command_hardware_interface);

    mobile_info.geometry.tracks.thickness=0.1;
    mobile_info.geometry.tracks.sprocketWheel.radius=0.4;

    controller_interface = std::make_unique<romea::ControllerInterface2TD>(mobile_info,joints_names);
    controller_interface->register_loaned_command_interfaces(command_loaned_interfaces);
    controller_interface->register_loaned_state_interfaces(state_loaned_interfaces);
  }

  std::shared_ptr<rclcpp::Node> node;

  std::vector<double> state_values;
  std::vector<double> command_values;
  std::vector<hardware_interface::StateInterface> state_hardware_interfaces;
  std::vector<hardware_interface::CommandInterface> command_hardware_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_loaned_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> command_loaned_interfaces;

  romea::MobileBaseInfo2TD  mobile_info;
  std::vector<std::string> joints_names;
  std::unique_ptr<romea::ControllerInterface2TD> controller_interface;

};


TEST_F(TestControllerInterface2TD, checkStateInterfaceNames)
{
  auto state_interface_names =controller_interface->get_state_interface_names();
  EXPECT_STREQ(state_interface_names[0].c_str(),"J1/velocity");
  EXPECT_STREQ(state_interface_names[1].c_str(),"J2/velocity");
}

TEST_F(TestControllerInterface2TD, checkCommandInterfaceNames)
{
  auto command_interface_names =controller_interface->get_command_interface_names();
  EXPECT_STREQ(command_interface_names[0].c_str(),"J1/velocity");
  EXPECT_STREQ(command_interface_names[1].c_str(),"J2/velocity");
}

TEST_F(TestControllerInterface2TD, checkSetCommand)
{
  romea::OdometryFrame2WD command;
  command.leftWheelSpeed=1;
  command.rightWheelSpeed=2;

  controller_interface->set_command(command);
  EXPECT_EQ(command_values[0],2);
  EXPECT_EQ(command_values[1],4);
}

TEST_F(TestControllerInterface2TD, checkGetMeasurement)
{
   state_values[0]=2;
   state_values[1]=4;

   auto measure = controller_interface->get_odometry_frame();
   EXPECT_EQ(measure.leftWheelSpeed,1);
   EXPECT_EQ(measure.rightWheelSpeed,2);
}
