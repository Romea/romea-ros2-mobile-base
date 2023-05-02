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
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

// local
#include "../test/test_helper.h"

std::string interface_name(const std::string & vehicle_type)
{
  return "test_gazebo_interface" + vehicle_type;
}

std::string xacro_filename(const std::string & vehicle_type)
{
  return std::string(TEST_DIR) + "/" + interface_name(vehicle_type) + ".xacro";
}

std::string urdf_filename(const std::string & vehicle_type)
{
  return "/tmp/" + interface_name(vehicle_type) + ".urdf";
}

std::string sdf_filename(const std::string & vehicle_type)
{
  return "/tmp/" + interface_name(vehicle_type) + ".sdf";
}

void create_urdf_file(const std::string & gazebo_interface_type, const std::string & vehicle_type)
{
  std::string create_urdf = "xacro " + xacro_filename(gazebo_interface_type) + " type:=" +
    vehicle_type + " > " + urdf_filename(vehicle_type);
  std::cout << create_urdf << std::endl;
  std::system(create_urdf.c_str());
}

void create_urdf_file(const std::string & vehicle_type)
{
  create_urdf_file(vehicle_type, vehicle_type);
}

void create_sdf_file(const std::string & vehicle_type)
{
  std::string create_sdf = "gz sdf -p " + urdf_filename(vehicle_type) + " > " + sdf_filename(
    vehicle_type);
  std::cout << create_sdf << std::endl;
  std::system(create_sdf.c_str());
}

std::string make_urdf_description(
  const std::string & gazebo_interface_type,
  const std::string & vehicle_type)
{
  std::cout << gazebo_interface_type << " " << vehicle_type << std::endl;
  create_urdf_file(gazebo_interface_type, vehicle_type);
  std::ifstream urdf_file(urdf_filename(vehicle_type));
  std::stringstream urdf_content;
  urdf_content << urdf_file.rdbuf();
  return urdf_content.str();
}

std::string make_urdf_description(const std::string & vehicle_type)
{
  return make_urdf_description(vehicle_type, vehicle_type);
}


std::string make_sdf_description(const std::string & vehicle_type)
{
  create_sdf_file(vehicle_type);
  std::ifstream sdf_file(sdf_filename(vehicle_type));
  std::stringstream sdf_content;
  sdf_content << sdf_file.rdbuf();
  return sdf_content.str();
}

#endif    // TEST_UTILS_HPP_
