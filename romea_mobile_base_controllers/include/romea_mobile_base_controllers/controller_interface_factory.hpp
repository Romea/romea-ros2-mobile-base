#ifndef _romea_ControllerInterfaceFactory_hpp_
#define _romea_ControllerInterfaceFactory_hpp_

#include "mobile_base_controller_interface1FAS2FWD.hpp"
#include "mobile_base_controller_interface1FAS2RWD.hpp"
#include "mobile_base_controller_interface1FWS2RWD.hpp"
#include "mobile_base_controller_interface2AS4WD.hpp"
#include "mobile_base_controller_interface2FWS2FWD.hpp"
#include "mobile_base_controller_interface2FWS2RWD.hpp"
#include "mobile_base_controller_interface2FWS4WD.hpp"
#include "mobile_base_controller_interface2WD.hpp"
#include "mobile_base_controller_interface4WD.hpp"
#include "mobile_base_controller_interface4WS4WD.hpp"

namespace romea
{

template<typename ExtractorType>
typename ExtractorType::Parameters load_interface_params(NodeParameters & robot_parameters)
{
  typename ExtractorType::Parameters parameters;
  load_interface_configuration(robot_parameters,parameters);
  return parameters;
}

template<typename ExtractorType,typename ...Args >
std::unique_ptr<ExtractorType> makeInterface(NodeParameters & robot_parameters,
                                             Args&&... args)
{
  auto parameters = load_interface_params<ExtractorType>(robot_parameters);
  return std::make_unique<ExtractorType>(std::forward<Args>(args)...,parameters);
}

}



#endif
