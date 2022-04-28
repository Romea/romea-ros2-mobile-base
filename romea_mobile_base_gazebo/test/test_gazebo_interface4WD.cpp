//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_gazebo/gazebo_system_interface.hpp"
#include <hardware_interface/component_parser.hpp>

class TestGazeboInterface4WD : public ::testing::Test
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
//    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface4WD.xacro";
//    std::string urdf_file =  std::string(TEST_DIR)+"/test_hardware_interface4WD.urdf";
//    std::string cmd = "xacro "+xacro_file + " > " + urdf_file;
//    std::system(cmd.c_str());

//    std::ifstream file(urdf_file.c_str());
//    std::stringstream buffer;
//    buffer << file.rdbuf();
//    std::cout << buffer.str() <<std::endl;

//    std::cout << " parse" << std::endl;
//    auto info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
//    std::cout << info.size()<< std::endl;

//    interface = std::make_unique<romea::HardwareInterface4WD>(info[0],hardware_interface::HW_IF_VELOCITY);
//    rclcpp::NodeOptions no;
//    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_command_limits_parameters.yaml"});
//    node = std::make_shared<rclcpp::Node>("test_command_limits_paramerters", no);
  }

//  std::shared_ptr<rclcpp::Node> node;

//  std::unique_ptr<romea::HardwareInterface4WD> interface;


};


TEST_F(TestGazeboInterface4WD, Toto)
{
  gazebo_ros2_control::GazeboSystemInterface * ptr = new romea::GazeboSystemInterface4WD;

}

