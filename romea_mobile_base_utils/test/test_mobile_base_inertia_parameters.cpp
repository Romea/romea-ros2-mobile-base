//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp>

class TestMobileBaseInertiaParams : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_mobile_base_inertia_paramerters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseInertiaParams, GetFullDescription)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_inertia_parameters.yaml");

  romea::declare_inertia_info(node,"inertia");
  romea::MobileBaseInertia inertia = romea::get_inertia_info(node,"inertia");
  EXPECT_DOUBLE_EQ(inertia.mass,1);
  EXPECT_DOUBLE_EQ(inertia.center.x(),2);
  EXPECT_DOUBLE_EQ(inertia.center.y(),3);
  EXPECT_DOUBLE_EQ(inertia.center.z(),4);
  EXPECT_DOUBLE_EQ(inertia.zMoment,5);
}


TEST_F(TestMobileBaseInertiaParams, GetDescriptionWithoutCenterPosition)
{
  loadYaml(std::string(TEST_DIR)+"/test_inertia_parameters.yaml");

  romea::declare_inertia_info(node,"inertia_miss_center");
  romea::MobileBaseInertia inertia = romea::get_inertia_info(node,"inertia_miss_center");
  EXPECT_DOUBLE_EQ(inertia.mass,1);
  EXPECT_DOUBLE_EQ(inertia.center.x(),0);
  EXPECT_DOUBLE_EQ(inertia.center.y(),0);
  EXPECT_DOUBLE_EQ(inertia.center.z(),0);
  EXPECT_DOUBLE_EQ(inertia.zMoment,5);
}



