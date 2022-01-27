//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/node.hpp>

//romea
#include <romea_odo_utils/kinematic_factory.hpp>

class TestConstraints : public ::testing::Test
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
      no.arguments({"--ros-args","--params-file","/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_odo/romea_odo_utils/test/data/constraints.yaml"});
      node = std::make_shared<rclcpp::Node>("test_constraints", no);
    }

    std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestConstraints, loadSkidSteeringConstraints)
{
//  romea::SkidSteeringConstraints constraints;
//  auto node_parameters = std::make_shared<romea::NodeParameters>(node,"skid_steering");
//  load_command_constraints(node_parameters,constraints);

  std::string bar;
  std::cout <<  "toto" << std::endl;
  node->declare_parameter("foo.bar");
 std::cout <<  "toto" << std::endl;
// std::cout << " bar " <<node->get_parameter("foo.bar",bar) << bar<< std::endl;

  auto sub_node = node->create_sub_node("foo");
  std::cout << " bar " <<sub_node->get_parameter<std::string>("bar",bar) << bar<< std::endl;

 std::cout <<  "toto" << std::endl;

//  EXPECT_DOUBLE_EQ(constraints.getMinimalLinearSpeed(),-1);
//  EXPECT_DOUBLE_EQ(constraints.getMaximalLinearSpeed(),2);
//  EXPECT_DOUBLE_EQ(constraints.getMaximalAbsoluteAngularSpeed(),0.5);
}



//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
