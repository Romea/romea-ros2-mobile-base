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

#ifndef ROMEA_MOBILE_BASE_GAZEBO__TEST_LEGACY__TEST_FIXTURE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__TEST_LEGACY__TEST_FIXTURE_HPP_

// std
#include <chrono>
#include <map>
#include <memory>
#include <string>

// gazebo
#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/transport/Node.hh>

// ros
#include "hardware_interface/component_parser.hpp"

template<typename InferfaceType>
class TestGazeboInterfaceFixture
{
public:
  TestGazeboInterfaceFixture(
    const std::string & _worldPath, const std::string & _robotUrdfDescription)
  : fixture(_worldPath), urdf(_robotUrdfDescription)
  {
  }

  virtual ~TestGazeboInterfaceFixture() = default;

  void Pause() { this->paused = true; }

  gz::sim::Server * Simulator()
  {
    if (!this->initialized) {
      this->fixture
        .OnConfigure([&](
                       const gz::sim::Entity & _entity,
                       const std::shared_ptr<const sdf::Element> & _sdf,
                       gz::sim::EntityComponentManager & _ecm,
                       gz::sim::EventManager & _eventManager) {
          gz::sim::World world(gz::sim::worldEntity(_ecm));
          auto physicsComponent =
            _ecm.Component<gz::sim::components::Physics>(world.Entity());  // NOLINT
          this->maxStepSize = physicsComponent->Data().MaxStepSize();
          this->OnConfigure(_entity, _sdf, _ecm, _eventManager);
        })
        .OnPreUpdate(
          [&](const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) {
            this->OnPreUpdate(_info, _ecm);
          })
        .OnPostUpdate(
          [&](const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm) {
            this->OnPostUpdate(_info, _ecm);
            this->info = _info;
          })
        .Finalize();
      this->initialized = true;
    }
    return this->fixture.Server().get();
  }

  uint64_t Step()
  {
    this->Simulator()->RunOnce(this->paused);
    return 1u;
  }

  uint64_t Step(uint64_t _steps)
  {
    static constexpr bool blocking = true;
    uint64_t initial_iterations = this->Iterations();
    this->Simulator()->Run(blocking, _steps, this->paused);
    return this->Iterations() - initial_iterations;
  }

  uint64_t Step(const std::chrono::steady_clock::duration & _step)
  {
    static constexpr bool blocking = true;
    uint64_t iterations = 0u;
    // Fetch simulator early to ensure it is initialized
    auto simulator = this->Simulator();
    const auto deadline = this->info.simTime + _step;
    do {
      const double stepSize = std::chrono::duration<double>(deadline - this->info.simTime).count();
      uint64_t previous_iterations = this->Iterations();
      simulator->Run(blocking, std::ceil(stepSize / this->maxStepSize), this->paused);
      iterations += this->Iterations() - previous_iterations;
    } while (this->info.simTime < deadline);
    return iterations;
  }

  uint64_t Iterations() const { return this->info.iterations; }

protected:
  virtual void OnConfigure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager &)
  {
    auto robotJoinEntities =
      _ecm.ChildrenByComponents(*_ecm.EntityByName("robot"), gz::sim::components::Joint());

    std::map<std::string, gz::sim::Entity> joints;
    for (const auto & jointEntity : robotJoinEntities) {
      const auto jointName = _ecm.Component<gz::sim::components::Name>(jointEntity)->Data();
      const auto * jointType = _ecm.Component<gz::sim::components::JointType>(jointEntity);
      if (jointType->Data() == sdf::JointType::REVOLUTE) {
        joints[jointName] = jointEntity;
      }
    }

    auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
    this->interface = std::make_unique<InferfaceType>(_ecm, joints, hardware_info[0], "velocity");
  }

  virtual void OnPreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) {}

  virtual void OnPostUpdate(const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &) {}

private:
  bool initialized{false};

  bool paused{false};

  double maxStepSize;

  gz::sim::UpdateInfo info;

  gz::sim::TestFixture fixture;

  std::string urdf;

public:
  std::unique_ptr<InferfaceType> interface;
};

#endif  // ROMEA_MOBILE_BASE_GAZEBO__TEST_LEGACY__TEST_FIXTURE_HPP_
