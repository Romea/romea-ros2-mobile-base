//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2RWD.hpp"

class TestControllerInterface1FAS2RWD : public ::testing::Test
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
                  "-p","joints.front_axle_steering_joint_name:=J1",
                  "-p","joints.rear_left_wheel_spinning_joint_name:=J2",
                  "-p","joints.rear_right_wheel_spinning_joint_name:=J3"
                 });

    node = std::make_shared<rclcpp::Node>("test_interface_controller_1FAS2RWD", no);

    state_values.resize(3);
    command_values.resize(3);

    romea::ControllerInterface1FAS2RWD::declare_joints_names(node,"joints");
    joints_names = romea::ControllerInterface1FAS2RWD::get_joints_names(node,"joints");

    state_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_POSITION,&state_values[0]);
    state_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_VELOCITY,&state_values[1]);
    state_hardware_interfaces.emplace_back(joints_names[2],hardware_interface::HW_IF_VELOCITY,&state_values[2]);

    command_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_POSITION,&command_values[0]);
    command_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_VELOCITY,&command_values[1]);
    command_hardware_interfaces.emplace_back(joints_names[2],hardware_interface::HW_IF_VELOCITY,&command_values[2]);

    for(auto & state_hardware_interface : state_hardware_interfaces)
      state_loaned_interfaces.emplace_back(state_hardware_interface);

    for(auto & command_hardware_interface : command_hardware_interfaces)
      command_loaned_interfaces.emplace_back(command_hardware_interface);

    mobile_info.geometry.rearAxle.wheels.radius=0.5;
    mobile_info.geometry.frontAxle.wheels.radius=0.5;

    controller_interface = std::make_unique<romea::ControllerInterface1FAS2RWD>(mobile_info);
  }

  std::shared_ptr<rclcpp::Node> node;

  std::vector<double> state_values;
  std::vector<double> command_values;
  std::vector<hardware_interface::StateInterface> state_hardware_interfaces;
  std::vector<hardware_interface::CommandInterface> command_hardware_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_loaned_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> command_loaned_interfaces;

  romea::MobileBaseInfo1FAS2RWD  mobile_info;
  std::vector<std::string> joints_names;
  std::unique_ptr<romea::ControllerInterface1FAS2RWD> controller_interface;

};


TEST_F(TestControllerInterface1FAS2RWD, checkStateInterfaceNames)
{
  auto hardware_interface_names = romea::ControllerInterface1FAS2RWD::hardware_interface_names(joints_names);
  EXPECT_STREQ(hardware_interface_names[0].c_str(),"J1/position");
  EXPECT_STREQ(hardware_interface_names[1].c_str(),"J2/velocity");
  EXPECT_STREQ(hardware_interface_names[2].c_str(),"J3/velocity");
}

TEST_F(TestControllerInterface1FAS2RWD, checkWrite)
{
  romea::OdometryFrame1FAS2RWD command;
  command.frontAxleSteeringAngle=11;
  command.rearLeftWheelSpeed=3;
  command.rearRightWheelSpeed=4;

  controller_interface->write(command,command_loaned_interfaces);
  EXPECT_EQ(command_values[0],11);
  EXPECT_EQ(command_values[1],6);
  EXPECT_EQ(command_values[2],8);
}

TEST_F(TestControllerInterface1FAS2RWD, checkRead)
{
  state_values[0]=11;
  state_values[1]=6;
  state_values[2]=8;

  romea::OdometryFrame1FAS2RWD measure;
  controller_interface->read(state_loaned_interfaces,measure);
  EXPECT_EQ(measure.frontAxleSteeringAngle,11);
  EXPECT_EQ(measure.rearLeftWheelSpeed,3);
  EXPECT_EQ(measure.rearRightWheelSpeed,4);
}

