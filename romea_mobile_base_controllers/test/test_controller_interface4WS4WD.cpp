//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_controllers/interfaces/controller_interface4WS4WD.hpp"

class TestControllerInterface4WS4WD : public ::testing::Test
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
                  "-p","joints.front_left_wheel_steering_joint_name:=J1",
                  "-p","joints.front_right_wheel_steering_joint_name:=J2",
                  "-p","joints.rear_left_wheel_steering_joint_name:=J3",
                  "-p","joints.rear_right_wheel_steering_joint_name:=J4",
                  "-p","joints.front_left_wheel_spinning_joint_name:=J5",
                  "-p","joints.front_right_wheel_spinning_joint_name:=J6",
                  "-p","joints.rear_left_wheel_spinning_joint_name:=J7",
                  "-p","joints.rear_right_wheel_spinning_joint_name:=J8"
                 });

    node = std::make_shared<rclcpp::Node>("test_interface_controller_4WS4WD", no);

    state_values.resize(8);
    command_values.resize(8);

    romea::ControllerInterface4WS4WD::declare_joints_names(node,"joints");
    joints_names = romea::ControllerInterface4WS4WD::get_joints_names(node,"joints");

    state_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_POSITION,&state_values[0]);
    state_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_POSITION,&state_values[1]);
    state_hardware_interfaces.emplace_back(joints_names[2],hardware_interface::HW_IF_POSITION,&state_values[2]);
    state_hardware_interfaces.emplace_back(joints_names[3],hardware_interface::HW_IF_POSITION,&state_values[3]);
    state_hardware_interfaces.emplace_back(joints_names[4],hardware_interface::HW_IF_VELOCITY,&state_values[4]);
    state_hardware_interfaces.emplace_back(joints_names[5],hardware_interface::HW_IF_VELOCITY,&state_values[5]);
    state_hardware_interfaces.emplace_back(joints_names[6],hardware_interface::HW_IF_VELOCITY,&state_values[6]);
    state_hardware_interfaces.emplace_back(joints_names[7],hardware_interface::HW_IF_VELOCITY,&state_values[7]);

    command_hardware_interfaces.emplace_back(joints_names[0],hardware_interface::HW_IF_POSITION,&command_values[0]);
    command_hardware_interfaces.emplace_back(joints_names[1],hardware_interface::HW_IF_POSITION,&command_values[1]);
    command_hardware_interfaces.emplace_back(joints_names[2],hardware_interface::HW_IF_POSITION,&command_values[2]);
    command_hardware_interfaces.emplace_back(joints_names[3],hardware_interface::HW_IF_POSITION,&command_values[3]);
    command_hardware_interfaces.emplace_back(joints_names[4],hardware_interface::HW_IF_VELOCITY,&command_values[4]);
    command_hardware_interfaces.emplace_back(joints_names[5],hardware_interface::HW_IF_VELOCITY,&command_values[5]);
    command_hardware_interfaces.emplace_back(joints_names[6],hardware_interface::HW_IF_VELOCITY,&command_values[6]);
    command_hardware_interfaces.emplace_back(joints_names[7],hardware_interface::HW_IF_VELOCITY,&command_values[7]);

    for(auto & state_hardware_interface : state_hardware_interfaces)
      state_loaned_interfaces.emplace_back(state_hardware_interface);

    for(auto & command_hardware_interface : command_hardware_interfaces)
      command_loaned_interfaces.emplace_back(command_hardware_interface);

    mobile_info.geometry.rearAxle.wheels.radius=0.5;
    mobile_info.geometry.frontAxle.wheels.radius=0.5;

    controller_interface = std::make_unique<romea::ControllerInterface4WS4WD>(mobile_info,joints_names);
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

  romea::MobileBaseInfo4WS4WD  mobile_info;
  std::vector<std::string> joints_names;
  std::unique_ptr<romea::ControllerInterface4WS4WD> controller_interface;

};


TEST_F(TestControllerInterface4WS4WD, checkStateInterfaceNames)
{
  auto state_interface_names =controller_interface->get_state_interface_names();
  EXPECT_STREQ(state_interface_names[0].c_str(),"J1/position");
  EXPECT_STREQ(state_interface_names[1].c_str(),"J2/position");
  EXPECT_STREQ(state_interface_names[2].c_str(),"J3/position");
  EXPECT_STREQ(state_interface_names[3].c_str(),"J4/position");
  EXPECT_STREQ(state_interface_names[4].c_str(),"J5/velocity");
  EXPECT_STREQ(state_interface_names[5].c_str(),"J6/velocity");
  EXPECT_STREQ(state_interface_names[6].c_str(),"J7/velocity");
  EXPECT_STREQ(state_interface_names[7].c_str(),"J8/velocity");
}

TEST_F(TestControllerInterface4WS4WD, checkCommandInterfaceNames)
{
  auto command_interface_names =controller_interface->get_command_interface_names();
  EXPECT_STREQ(command_interface_names[0].c_str(),"J1/position");
  EXPECT_STREQ(command_interface_names[1].c_str(),"J2/position");
  EXPECT_STREQ(command_interface_names[2].c_str(),"J3/position");
  EXPECT_STREQ(command_interface_names[3].c_str(),"J4/position");
  EXPECT_STREQ(command_interface_names[4].c_str(),"J5/velocity");
  EXPECT_STREQ(command_interface_names[5].c_str(),"J6/velocity");
  EXPECT_STREQ(command_interface_names[6].c_str(),"J7/velocity");
  EXPECT_STREQ(command_interface_names[7].c_str(),"J8/velocity");
}
TEST_F(TestControllerInterface4WS4WD, checkSetCommand)
{
  romea::OdometryFrame4WS4WD command;
  command.frontLeftWheelAngle=11;
  command.frontRightWheelAngle=12;
  command.rearLeftWheelAngle=13;
  command.rearRightWheelAngle=14;
  command.frontLeftWheelSpeed=1;
  command.frontRightWheelSpeed=2;
  command.rearLeftWheelSpeed=3;
  command.rearRightWheelSpeed=4;

  controller_interface->set_command(command);
  EXPECT_EQ(command_values[0],11);
  EXPECT_EQ(command_values[1],12);
  EXPECT_EQ(command_values[2],13);
  EXPECT_EQ(command_values[3],14);
  EXPECT_EQ(command_values[4],2);
  EXPECT_EQ(command_values[5],4);
  EXPECT_EQ(command_values[6],6);
  EXPECT_EQ(command_values[7],8);

}

TEST_F(TestControllerInterface4WS4WD, checkGetMeasurement)
{
  state_values[0]=11;
  state_values[1]=12;
  state_values[2]=13;
  state_values[3]=14;
  state_values[4]=2;
  state_values[5]=4;
  state_values[6]=6;
  state_values[7]=8;

  auto measure = controller_interface->get_odometry_frame();
  EXPECT_EQ(measure.frontLeftWheelAngle,11);
  EXPECT_EQ(measure.frontRightWheelAngle,12);
  EXPECT_EQ(measure.rearLeftWheelAngle,13);
  EXPECT_EQ(measure.rearRightWheelAngle,14);
  EXPECT_EQ(measure.frontLeftWheelSpeed,1);
  EXPECT_EQ(measure.frontRightWheelSpeed,2);
  EXPECT_EQ(measure.rearLeftWheelSpeed,3);
  EXPECT_EQ(measure.rearRightWheelSpeed,4);
}

