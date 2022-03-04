#ifndef _romea_HardwareSystemInterface_hpp_
#define _romea_HardwareSystemInterface_hpp_

//romea
#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FWS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"


//ros
#include <hardware_interface/system_interface.hpp>


namespace romea
{

template <typename HardwareInterface>
class HardwareSystemInterface : public hardware_interface::SystemInterface{


public:

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:

  HardwareSystemInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override ;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

protected :

  std::unique_ptr<HardwareInterface> hardware_interface_;
};

using HardwareSystemInterfac1FAS2FWD = HardwareSystemInterface<HardwareInterface1FAS2FWD>;
using HardwareSystemInterfac1FAS2RWD = HardwareSystemInterface<HardwareInterface1FAS2RWD>;
using HardwareSystemInterface1FWS2RWD = HardwareSystemInterface<HardwareInterface1FWS2RWD>;
using HardwareSystemInterface2AS4WD = HardwareSystemInterface<HardwareInterface2AS4WD>;
using HardwareSystemInterface2FWS2FWD = HardwareSystemInterface<HardwareInterface2FWS2FWD>;
using HardwareSystemInterface2FWS2RWD = HardwareSystemInterface<HardwareInterface2FWS2RWD>;
using HardwareSystemInterface2FWS4WD = HardwareSystemInterface<HardwareInterface2FWS4WD>;
using HardwareSystemInterface2WD = HardwareSystemInterface<HardwareInterface2WD>;
using HardwareSystemInterface4WD = HardwareSystemInterface<HardwareInterface4WD>;
using HardwareSystemInterface4WS4WD = HardwareSystemInterface<HardwareInterface4WS4WD>;

}

#endif