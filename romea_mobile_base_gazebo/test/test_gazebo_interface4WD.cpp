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
#include "romea_mobile_base_gazebo/gazebo_interface4WD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestGazeboInterface4WD : public gazebo::ServerFixture{};


TEST_F(TestGazeboInterface4WD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("4WD");
  std::string sdf_description = make_sdf_description("4WD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface4WD gazebo_interface(GetModel("robot"),hardware_info[0],"velocity");

  romea::SimulationCommand4WD command = {-1.0,1.0,-2.0,2.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(command.frontLeftWheelSetPoint,state.frontLeftWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.frontRightWheelSetPoint,state.frontRightWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rearLeftWheelSetPoint,state.rearLeftWheelSpinMotion.velocity,0.1);
  EXPECT_NEAR(command.rearRightWheelSetPoint,state.rearRightWheelSpinMotion.velocity,0.1);
}

