// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// local
#include "mobile_base_parameters1FAS2FWD.hpp"
#include "mobile_base_parameters1FAS2RWD.hpp"
#include "mobile_base_parameters2AS4WD.hpp"
#include "mobile_base_parameters2FWS2FWD.hpp"
#include "mobile_base_parameters2FWS2RWD.hpp"
#include "mobile_base_parameters2FWS4WD.hpp"
#include "mobile_base_parameters2TD.hpp"
#include "mobile_base_parameters2WD.hpp"
#include "mobile_base_parameters4WD.hpp"
#include "mobile_base_parameters4WS4WD.hpp"


namespace romea
{


template<typename MobileBaseInfo, typename Node>
void declare_mobile_base_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo1FAS2FWD>) {
    declare_mobile_base_info_1FAS2FWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo1FAS2RWD>) {
    declare_mobile_base_info_1FAS2RWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2AS4WD>) {
    declare_mobile_base_info_2AS4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS2FWD>) {
    declare_mobile_base_info_2FWS2FWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS2RWD>) {
    declare_mobile_base_info_2FWS2RWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS4WD>) {
    declare_mobile_base_info_2FWS4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2TD>) {
    declare_mobile_base_info_2TD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2WD>) {
    declare_mobile_base_info_2WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo4WD>) {
    declare_mobile_base_info_4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo4WS4WD>) {
    declare_mobile_base_info_4WS4WD(node, parameters_ns);
  }
}


template<typename MobileBaseInfo, typename Node>
MobileBaseInfo get_mobile_base_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo1FAS2FWD>) {
    return get_mobile_base_info_1FAS2FWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo1FAS2RWD>) {
    return get_mobile_base_info_1FAS2RWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2AS4WD>) {
    return get_mobile_base_info_2AS4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS2FWD>) {
    return get_mobile_base_info_2FWS2FWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS2RWD>) {
    return get_mobile_base_info_2FWS2RWD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2FWS4WD>) {
    return get_mobile_base_info_2FWS4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2TD>) {
    return get_mobile_base_info_2TD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo2WD>) {
    return get_mobile_base_info_2WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo4WD>) {
    return get_mobile_base_info_4WD(node, parameters_ns);
  } else if constexpr (std::is_same_v<MobileBaseInfo, MobileBaseInfo4WS4WD>) {
    return get_mobile_base_info_4WS4WD(node, parameters_ns);
  }
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS_HPP_
