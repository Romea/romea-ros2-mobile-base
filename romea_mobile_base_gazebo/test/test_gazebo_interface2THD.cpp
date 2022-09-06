//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include "test_utils.hpp"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//gazebo
#include <gazebo/test/ServerFixture.hh>

//romea
#include "romea_mobile_base_gazebo/gazebo_interface2THD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestGazeboInterface2ThD : public gazebo::ServerFixture{};


TEST_F(TestGazeboInterface2ThD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2THD");
  std::string sdf_description = make_sdf_description("2THD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2THD gazebo_interface(GetModel("robot"),hardware_info[0],"velocity");

  romea::SimulationCommand2THD command = {-1.0,1.0,-2.0,2.0,3.0,-3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(command.leftSprocketWheelSetPoint,state.leftSprocketWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rightSprocketWheelSetPoint,state.rightSprocketWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.frontLeftIdlerWheelSetPoint,state.frontLeftIdlerWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.frontRightIdlerWheelSetPoint,state.frontRightIdlerWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rearLeftIdlerWheelSetPoint,state.rearLeftIdlerWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rearRightIdlerWheelSetPoint,state.rearRightIdlerWheelSpinMotion.velocity,0.1);

}

