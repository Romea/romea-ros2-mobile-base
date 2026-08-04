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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

// std
#include <cassert>
#include <fstream>
#include <sstream>
#include <string>

// test
#include "test_helper.h"  // NOLINT

// tinyxml2
#include "tinyxml2.h"  // NOLINT

std::string empty_world_filename()
{
  return std::string(TEST_LEGACY_DIR) + "/gz_empty.sdf";
}

std::string xacro_filename(const std::string & name)
{
  return std::string(TEST_DIR) + "/" + name + ".xacro";
}

std::string urdf_filename(const std::string & name)
{
  return "/tmp/" + name + ".urdf";
}

std::string sdf_filename(const std::string & name)
{
  return "/tmp/" + name + ".sdf";
}

std::string world_filename(const std::string & name)
{
  return "/tmp/" + name + "_world.sdf";
}

void create_urdf_file(const std::string & name)
{
  const std::string command = "xacro " + xacro_filename(name) + " > " + urdf_filename(name);
  std::system(command.c_str());
}

void create_sdf_file(const std::string & name)
{
  const std::string command = "gz sdf -p " + urdf_filename(name) + " > " + sdf_filename(name);
  std::system(command.c_str());
}

std::string make_urdf_description(const std::string & name)
{
  create_urdf_file(name);

  std::ifstream urdf_file(urdf_filename(name));
  std::stringstream urdf_content;
  urdf_content << urdf_file.rdbuf();
  return urdf_content.str();
}

std::string create_sdf_world_file(const std::string & name)
{
  create_sdf_file(name);

  tinyxml2::XMLDocument world;
  world.LoadFile(empty_world_filename().c_str());
  tinyxml2::XMLElement * world_element =
    world.FirstChildElement("sdf")->FirstChildElement("world");
  assert(world_element != nullptr);

  tinyxml2::XMLDocument robot;
  robot.LoadFile(sdf_filename(name).c_str());
  tinyxml2::XMLElement * robot_element = robot.FirstChildElement("sdf")->FirstChildElement("model");
  assert(robot_element != nullptr);

  tinyxml2::XMLNode * cloned_model = robot_element->DeepClone(&world);
  world_element->InsertEndChild(cloned_model->ToElement());

  world.SaveFile(world_filename(name).c_str());
  return world_filename(name);
}

#endif  // TEST_UTILS_HPP_
