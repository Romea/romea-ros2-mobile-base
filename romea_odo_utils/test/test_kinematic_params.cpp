//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_odo_utils/kinematic_factory.hpp>

class TestKinematicParams : public ::testing::Test
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
  }

  void loadYaml(const std::string & config_filename)
  {
    rclcpp::NodeOptions no;
    no.arguments({"--ros-args","--params-file",config_filename});
    node = std::make_shared<rclcpp::Node>("test_kinematic_params", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestKinematicParams, load4WDtoSkidSteeringKinematicParams)
{
  loadYaml("/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/4WD_params.yaml");

  romea::NodeParameters node_parameters(node);
  romea::SkidSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters,kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelTrack,0.515);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelSpeed,3);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration,1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance,0.1*0.1);
}


TEST_F(TestKinematicParams, load4WS4WDtoFourWheelSteeringKinematicParams)
{
  loadYaml("/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/4WS4WD_params.yaml");

  romea::NodeParameters node_parameters(node);
  romea::FourWheelSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters,kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase,1.2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase,0.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelTrack,0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.hubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelSpeed,1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration,1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngle,0.26180);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngularSpeed,0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance,0.1*0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelAngleVariance,0.017453*0.017453);
}


TEST_F(TestKinematicParams, load4WS4WDtoTwoAxleKinematicParams) {
  loadYaml("/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/4WS4WD_params.yaml");

  romea::NodeParameters node_parameters(node);
  romea::TwoAxleSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters,kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase,1.2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase,0.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack,0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack,0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed,1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed,1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration,1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalSteeringAngle,0.26180);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalSteeringAngle,0.26180);
  //  EXPECT_DOUBLE_EQ(parameters.maximalSteeringAngularSpeed,0.017453*0.017453);
}

TEST_F(TestKinematicParams, load2WS4WDtoTwoWheelSteeringKinematicParams) {
  loadYaml("/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/2FWS4WD_params.yaml");

  romea::NodeParameters node_parameters(node);
  romea::TwoWheelSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters,kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase,2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase,0);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack,1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack,1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed,2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalWheelSpeed,2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration,1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngle,0.93);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngularSpeed,0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance,0.1*0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelAngleVariance,0.017453*0.017453);
}

TEST_F(TestKinematicParams, load2WS4WDtoOneSteeringKinematicParams) {
  loadYaml("/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/2FWS4WD_params.yaml");

  romea::NodeParameters node_parameters(node);
  romea::OneAxleSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters,kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase,2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase,0);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack,1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack,1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset,0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed,2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalWheelSpeed,2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration,1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalSteeringAngle,0.72850784045657924);
  //  EXPECT_DOUBLE_EQ(parameters.maximalSteeringAngularSpeed,0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance,0.1*0.1);
  //  EXPECT_DOUBLE_EQ(parameters.steeringAngleVariance,0.017453*0.017453);

}

//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
