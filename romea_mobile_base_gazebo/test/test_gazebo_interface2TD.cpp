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
#include "romea_mobile_base_gazebo/gazebo_interface2TD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestGazeboInterface2TD : public gazebo::ServerFixture{};


TEST_F(TestGazeboInterface2TD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2TD");
  std::string sdf_description = make_sdf_description("2TD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2TD gazebo_interface(GetModel("robot"),hardware_info[0],"velocity");

  romea::SimulationCommand2TD command = {-1.0,1.0,-2.0,2.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(command.leftSprocketWheelSetPoint,state.leftSprocketWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rightSprocketWheelSetPoint,state.rightSprocketWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.leftIdlerWheelSetPoint,state.leftIdlerWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rightIdlerWheelSetPoint,state.rightIdlerWheelSpinMotion.velocity,0.1);
}

