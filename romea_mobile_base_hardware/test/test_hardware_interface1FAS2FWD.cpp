//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInterface1FAS2FWD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_hardware_interface1FAS2FWD.xacro";
    std::string urdf_file =  std::string(TEST_DIR)+"/test_hardware_interface1FAS2FWD.urdf";
    std::string cmd = "xacro "+xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    auto info = hardware_interface::parse_control_resources_from_urdf(urdf_file);
    std::cout << info.size()<< std::endl;
//    rclcpp::NodeOptions no;
//    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_command_limits_parameters.yaml"});
//    node = std::make_shared<rclcpp::Node>("test_command_limits_paramerters", no);
  }

//  std::shared_ptr<rclcpp::Node> node;

  std::unique_ptr<romea::HardwareInterface1FAS2FWD> interface;
};


TEST_F(TestHarwareInterface1FAS2FWD, Toto)
{

}

