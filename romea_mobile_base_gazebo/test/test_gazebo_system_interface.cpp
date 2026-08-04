// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

// gazebo
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Physics.hh>

// gtest
#include "gtest/gtest.h"

// ros
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_gazebo/gazebo_system_interface.hpp"

// test
#include "test_utils.hpp"

class TestGazeboSystemInterfaceFixture
{
public:
  TestGazeboSystemInterfaceFixture(
    const std::string & world_path, const std::string & urdf_description)
  : fixture_(world_path), urdf_(urdf_description)
  {
  }

  gz::sim::Server * Simulator()
  {
    if (!initialized_) {
      fixture_
        .OnConfigure([&](
                       const gz::sim::Entity &,
                       const std::shared_ptr<const sdf::Element> &,
                       gz::sim::EntityComponentManager & ecm,
                       gz::sim::EventManager &) {
          auto world = gz::sim::World(gz::sim::worldEntity(ecm));
          auto physics = ecm.Component<gz::sim::components::Physics>(world.Entity());
          max_step_size_ = physics->Data().MaxStepSize();

          ecm_ = &ecm;
          const auto joint_entities =
            ecm.ChildrenByComponents(*ecm.EntityByName("robot"), gz::sim::components::Joint());
          for (const auto & joint_entity : joint_entities) {
            const auto joint_name = ecm.Component<gz::sim::components::Name>(joint_entity)->Data();
            joints_[joint_name] = joint_entity;
          }

          auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_);
          node_ = rclcpp::Node::make_shared("test_gazebo_system_interface");
          interface_ = std::make_unique<romea::ros2::GazeboSystemInterface>();

          ASSERT_EQ(
            interface_->on_init(hardware_info[0]),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
          ASSERT_TRUE(interface_->initSim(node_, joints_, hardware_info[0], ecm, 1000));
        })
        .OnPostUpdate(
          [&](const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager &) {
            info_ = info;
          })
        .Finalize();
      initialized_ = true;
    }
    return fixture_.Server().get();
  }

  uint64_t Step()
  {
    Simulator()->RunOnce(false);
    return 1u;
  }

  uint64_t Step(uint64_t steps)
  {
    static constexpr bool blocking = true;
    const auto initial_iterations = Iterations();
    Simulator()->Run(blocking, steps, false);
    return Iterations() - initial_iterations;
  }

  uint64_t Iterations() const { return info_.iterations; }

  romea::ros2::GazeboSystemInterface & interface() { return *interface_; }

  double joint_velocity_command(const std::string & joint_name) const
  {
    const auto * command =
      ecm_->Component<gz::sim::components::JointVelocityCmd>(joints_.at(joint_name));
    return command->Data()[0];
  }

  void set_joint_velocity(const std::string & joint_name, const double & velocity)
  {
    ecm_->SetComponentData<gz::sim::components::JointVelocity>(joints_.at(joint_name), {velocity});
  }

  void set_joint_position(const std::string & joint_name, const double & position)
  {
    ecm_->SetComponentData<gz::sim::components::JointPosition>(joints_.at(joint_name), {position});
  }

private:
  bool initialized_{false};
  double max_step_size_{0.001};
  gz::sim::UpdateInfo info_;
  gz::sim::TestFixture fixture_;
  std::string urdf_;
  gz::sim::EntityComponentManager * ecm_{nullptr};
  std::map<std::string, gz::sim::Entity> joints_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<romea::ros2::GazeboSystemInterface> interface_;
};

TEST(TestGazeboSystemInterface, testWriteRead)
{
  const std::string test_name = "test_gazebo_system_interface";
  const std::string urdf = make_urdf_description(test_name);
  const std::string world = create_sdf_world_file(test_name);

  TestGazeboSystemInterfaceFixture fixture(world, urdf);
  fixture.Simulator();

  auto command_interfaces = fixture.interface().export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 8u);

  EXPECT_TRUE(command_interfaces[0].set_value(0.1));
  EXPECT_TRUE(command_interfaces[1].set_value(-0.2));
  EXPECT_TRUE(command_interfaces[2].set_value(0.3));
  EXPECT_TRUE(command_interfaces[3].set_value(-0.4));
  EXPECT_TRUE(command_interfaces[4].set_value(-1.0));
  EXPECT_TRUE(command_interfaces[5].set_value(1.0));
  EXPECT_TRUE(command_interfaces[6].set_value(-2.0));
  EXPECT_TRUE(command_interfaces[7].set_value(2.0));

  fixture.interface().write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  EXPECT_NEAR(fixture.joint_velocity_command("front_left_wheel_steering_joint"), 10.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("front_right_wheel_steering_joint"), -20.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("rear_left_wheel_steering_joint"), 30.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("rear_right_wheel_steering_joint"), -40.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("front_left_wheel_spinning_joint"), -1.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("front_right_wheel_spinning_joint"), 1.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("rear_left_wheel_spinning_joint"), -2.0, 0.001);
  EXPECT_NEAR(fixture.joint_velocity_command("rear_right_wheel_spinning_joint"), 2.0, 0.001);

  fixture.Step();

  fixture.set_joint_position("front_left_wheel_steering_joint", 0.11);
  fixture.set_joint_position("front_right_wheel_steering_joint", -0.22);
  fixture.set_joint_position("rear_left_wheel_steering_joint", 0.33);
  fixture.set_joint_position("rear_right_wheel_steering_joint", -0.44);
  fixture.set_joint_velocity("front_left_wheel_spinning_joint", -1.1);
  fixture.set_joint_velocity("front_right_wheel_spinning_joint", 1.1);
  fixture.set_joint_velocity("rear_left_wheel_spinning_joint", -2.1);
  fixture.set_joint_velocity("rear_right_wheel_spinning_joint", 2.1);
  fixture.interface().read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));

  auto state_interfaces = fixture.interface().export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 16u);

  EXPECT_NEAR(state_interfaces[0].get_value(), 0.11, 0.001);
  EXPECT_NEAR(state_interfaces[1].get_value(), -0.22, 0.001);
  EXPECT_NEAR(state_interfaces[2].get_value(), 0.33, 0.001);
  EXPECT_NEAR(state_interfaces[3].get_value(), -0.44, 0.001);
  EXPECT_NEAR(state_interfaces[5].get_value(), -1.1, 0.001);
  EXPECT_NEAR(state_interfaces[8].get_value(), 1.1, 0.001);
  EXPECT_NEAR(state_interfaces[11].get_value(), -2.1, 0.001);
  EXPECT_NEAR(state_interfaces[14].get_value(), 2.1, 0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
