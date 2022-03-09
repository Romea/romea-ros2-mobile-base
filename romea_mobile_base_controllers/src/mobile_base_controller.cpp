#include "romea_mobile_base_controllers/mobile_base_controller.hpp"
#include <romea_mobile_base_utils/conversions/command_conversions.hpp>
#include <romea_mobile_base_utils/conversions/kinematic_conversions.hpp>
#include <romea_mobile_base_utils/params/mobile_base_parameters.hpp>
//#include <romea_mobile_base_utils/kinematic_factory.hpp>
#include <romea_mobile_base_controller_interfaces/mobile_base_controller_interface_factory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace  {
const std::string DEFAULT_BASE_FRAME_ID = "base_link";
const std::string DEFAULT_ODOM_FRAME_ID = "odom";
const double DEFAULT_COMMAND_TIMEOUT = 0.5;
const double DEFAULT_PUBLISH_RATE = 50.0;
}

namespace romea{

template <typename OdometryFrameType, typename KinematicType>
MobileBaseController<OdometryFrameType,KinematicType>::MobileBaseController():
  ControllerInterface(),
  odometry_interface_(nullptr),
  kinematic_parameters_(),
  user_command_contraints_(),
  odometry_measure_(),
  kinematic_measure_(),
  odometry_frame_(),
  command_sub(),
  previous_command_(),
  current_command_(),
  command_buffer_(),
  update_time_(),
  last_state_publish_time_(),
  publish_period_(0,0),
  command_timeout_(0,0),
  dead_reckoning_publisher_(),
  odometry_measure_publisher_(),
  kinematic_measure_publisher_()
{
}


//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
declare_parameters_(std::shared_ptr<rclcpp::Node> node)
{
  declare_mobile_base_info<MobileBaseInfo>(node,"base_info");
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_init()
{
  return CallbackReturn::SUCCESS;
}


//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
controller_interface::InterfaceConfiguration
MobileBaseController<OdometryFrameType,KinematicType>::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
        odometry_interface_->getCommandInterfaceNames()};
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
controller_interface::InterfaceConfiguration
MobileBaseController<OdometryFrameType,KinematicType>::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
        odometry_interface_->getStateInterfaceNames()};
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_configure(const rclcpp_lifecycle::State & previous_state)
{

  NodeParameters node_parameters(node_);
  NodeParameters robot_parameters(node_,"robot");

  loadKinematicParams_(robot_parameters);
  loadCommandConstraints_(node_parameters);
  loadPublishPeriod_(node_parameters);
  loadCommandTimeout_(node_parameters);
  initInterface_(robot_parameters,node_parameters);
  initPublishers_(node_parameters);
  initCmdSubscriber_();

//  std::string joint_prefixs = node_parameters.loadParam
//      odometry_interface_ = makeInterface<OdometryInterface>(robot_parameters);
  //  try {
  //     OdometryInterface::Parameter interface_parameters;
  //     load_interface_configuration(robot_parameters,interface_parameters);
  //     odom
  //  }
  //  catch()
  //  {

  //  }

}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_activate(const rclcpp_lifecycle::State & previous_state)
{
//  is_halted = false;
//  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");

  return CallbackReturn::SUCCESS;

}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_deactivate(const rclcpp_lifecycle::State &)
{
//  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_cleanup(const rclcpp_lifecycle::State &)
{
  reset_();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_error(const rclcpp_lifecycle::State &)
{
  reset_();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
CallbackReturn MobileBaseController<OdometryFrameType,KinematicType>::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}



//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
controller_interface::return_type MobileBaseController<OdometryFrameType,KinematicType>::
update(const rclcpp::Time& time, const rclcpp::Duration & /*period*/)
{
    update_time_=time;
//    ROS_INFO_STREAM_NAMED(name_, "update_controller_state_");
    update_controller_state_();
//    ROS_INFO_STREAM_NAMED(name_, "publish_controller_state_");
    publish_controller_state_();
//    ROS_INFO_STREAM_NAMED(name_, "read command");
    current_command_ = *(command_buffer_.readFromRT());

    RCLCPP_INFO_STREAM(node_->get_logger(), "\n"<<current_command_.cmd);

//    RCLCPP_INFO_STREAM(node_->get_logger(), "times " << time <<" "<<current_command_.stamp <<" "<< previous_command_.stamp);


//    ROS_INFO_STREAM_NAMED(name_, "check timeout");
    if(timeout_())
    {
//        RCLCPP_INFO_STREAM(node_->get_logger(), "brake");
        brake_();
    }

//    RCLCPP_INFO_STREAM(node_->get_logger(), "check new command available");

    else if(new_command_available_())
    {

//        RCLCPP_INFO_STREAM(node_->get_logger(),"odometry frame measured");
//        RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<odometry_frame_);

//        RCLCPP_INFO_STREAM(node_->get_logger(), " new command ok");
//        RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<current_command_.cmd);

        clamp_current_command_();
        RCLCPP_INFO_STREAM(node_->get_logger(), "cnew clamp command");
        RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<current_command_.cmd);

        send_current_command_();
//        RCLCPP_INFO_STREAM(node_->get_logger(), "cooucou new command");

    }

    return controller_interface::return_type::OK;

}

  ////-----------------------------------------------------------------------------
//template <typename OdometryFrameType, typename KinematicType>
//void MobileBaseController<OdometryFrameType,KinematicType>::starting(const ros::Time& time)
//{
//    brake_();
//    previous_command_.cmd = Command();
//    previous_command_.stamp=time;
//    current_command_.cmd = Command();
//    current_command_.stamp=time;//    last_state_publish_time_=time;
//    dead_reckoning_publisher_.reset();
//}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
initInterface_(NodeParameters & node_paramters,
               NodeParameters & robot_parameters)
{
    std::string prefix = node_paramters.loadParamOr<std::string>("joints_prefix","");
    auto parameters = load_interface_params<OdometryInterface>(robot_parameters);

    odometry_interface_= std::make_unique<OdometryInterface>(parameters,
                                                             prefix,
                                                             command_interfaces_,
                                                             state_interfaces_);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
initCmdSubscriber_()
{
    std::string cmd_topic;
    if(std::is_same<Command, SkidSteeringCommand>::value)
    {
      cmd_topic="cmd_skid_steering";
    }
    else if(std::is_same<Command, OneAxleSteeringCommand>::value)
    {
       cmd_topic="cmd_one_axle_steering";
    }
    else if(std::is_same<Command, TwoAxleSteeringCommand>::value)
    {
        cmd_topic="cmd_two_axle_steering";
    }
    else
    {
        cmd_topic="cmd_omni_steering";

    }

    auto callback = std::bind(&MobileBaseController::command_callback_,this,std::placeholders::_1);
    node_->create_subscription<CommandMsg>(cmd_topic,1,callback);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
initPublishers_(NodeParameters & node_parameters)
{
    std::string base_frame_id = node_parameters.loadParamOr("base_frame_id", DEFAULT_BASE_FRAME_ID);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Base frame_id set to " << base_frame_id);

    std::string odom_frame_id=node_parameters.loadParamOr("odom_frame_id", DEFAULT_ODOM_FRAME_ID);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Odometry frame_id set to " << odom_frame_id);

    bool enable_odom_tf = node_parameters.loadParamOr("enable_odom_tf",false);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing to tf is " <<(enable_odom_tf?"enabled":"disabled"));

    odometry_measure_publisher_.init(node_,"odometry",base_frame_id,100);
    kinematic_measure_publisher_.init(node_,"kinematic",base_frame_id,100);
    dead_reckoning_publisher_.init(node_,odom_frame_id,base_frame_id,enable_odom_tf);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
loadKinematicParams_(NodeParameters & robot_parameters)
{
    load_kinematic_params(robot_parameters,kinematic_parameters_);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
loadCommandConstraints_(NodeParameters & node_parameters)
{
    loadCommandConstraints(node_parameters,user_command_contraints_);
    //    RCLCPP_INFO_STREAM(node_->get_logger(), "Command constraints are set to "<< user_command_contraints_);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
loadPublishPeriod_(NodeParameters & node_parameters)
{
    double publish_rate = node_parameters.loadParamOr("publish_rate", DEFAULT_PUBLISH_RATE);
//    RCLCPP_INFO_STREAM(node_->get_logger(), "Controller state will be published at "<< publish_rate << "Hz.");
    publish_period_ = rclcpp::Duration(1.0 / publish_rate);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::
loadCommandTimeout_(NodeParameters & node_parameters)
{
    double command_timeout = node_parameters.loadParamOr("cmd_timeout", DEFAULT_COMMAND_TIMEOUT);
//    RCLCPP_INFO_STREAM(node_->get_logger(), "Velocity commands will be considered old if they are older than "<< command_timeout << "s.");
    command_timeout_ = rclcpp::Duration(command_timeout);
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
bool MobileBaseController<OdometryFrameType,KinematicType>::timeout_()
{
//    RCLCPP_INFO_STREAM(node_->get_logger(), "dts " <<  update_time_ - current_command_.stamp <<" "<< command_timeout_);
//    RCLCPP_INFO_STREAM(node_->get_logger(), "dts " <<  (update_time_ - current_command_.stamp).toSec() <<" "<< command_timeout_.toSec());

    return update_time_ - current_command_.stamp>command_timeout_;
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
bool MobileBaseController<OdometryFrameType,KinematicType>::new_command_available_()
{
    return current_command_.stamp != previous_command_.stamp;
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::publish_controller_state_()
{
    if (last_state_publish_time_ + publish_period_ < update_time_)
    {
        last_state_publish_time_ += publish_period_;
        dead_reckoning_publisher_.update(update_time_,kinematic_measure_);
        odometry_measure_publisher_.publish(update_time_,odometry_measure_);
        kinematic_measure_publisher_.publish(update_time_,kinematic_measure_);
    }
}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::update_controller_state_()
{
    odometry_frame_ = odometry_interface_->getOdometryFrame();
//    RCLCPP_INFO_STREAM(node_->get_logger(),"odometry frame measured");
//    RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<odometry_frame_);

    inverseKinematic(kinematic_parameters_,odometry_frame_,odometry_measure_);
//    RCLCPP_INFO_STREAM(node_->get_logger(),"odometry measure");
//    RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<odometry_measure_);

    kinematic_measure_ = toKinematicMeasure(odometry_measure_,kinematic_parameters_);
//    RCLCPP_INFO_STREAM(node_->get_logger(),"kinmeatic measure");
//    RCLCPP_INFO_STREAM(node_->get_logger(),"\n"<<kinematic_measure_);

}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::clamp_current_command_()
{

    current_command_.cmd=clamp(kinematic_parameters_,
                               user_command_contraints_,
                               current_command_.cmd);



    //    if(kinematic_command_clamp_)
    //    {
    //        double dt = (current_command_.stamp-previous_command_.stamp).toSec();
    //        Command clamped_command = clamp(kinematic_parameters_,
    //                                        previous_command_.cmd,
    //                                        clamped_command_.cmd,
    //                                        dt);
    //   }

    previous_command_=current_command_;

}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::send_current_command_()
{
    forwardKinematic(kinematic_parameters_,current_command_.cmd,odometry_frame_);
    RCLCPP_INFO_STREAM(node_->get_logger(),"odometry frame commad");
    RCLCPP_INFO_STREAM(node_->get_logger(),odometry_frame_);

    odometry_interface_->setCommand(odometry_frame_);
}


//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::brake_()
{
    odometry_interface_->setCommand(OdometryFrameType());
}


////-----------------------------------------------------------------------------
//template <typename OdometryFrameType, typename KinematicType>
//void MobileBaseController<OdometryFrameType,KinematicType>::
//command_callback_(const typename CommandMsg::ConstPtr &cmd_msg)
//{

////    ROS_INFO_STREAM("command_callback_");
//    StampedCommand stamped_cmd;
//    to_romea(*cmd_msg,stamped_cmd.cmd);
//    stamped_cmd.stamp=ros::Time::now();

//    if (isRunning())
//    {

////        ROS_INFO_STREAM("running");
//        if(!isValid(stamped_cmd.cmd))
//        {
//            ROS_WARN_THROTTLE(1.0, "Received NaN in command. Ignoring.");
//            return;
//        }

////        ROS_INFO_STREAM("valid");
////        ROS_INFO_STREAM(stamped_cmd.cmd);
//        command_buffer_.writeFromNonRT(stamped_cmd);
//    }
//    else
//    {
//        ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
//    }
//}

//-----------------------------------------------------------------------------
template <typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType,KinematicType>::reset_()
{
  command_sub.reset();
  dead_reckoning_publisher_.reset();

  brake_();
  odometry_interface_.reset();

}


template class MobileBaseController<OdometryFrame4WD,SkidSteeringKinematic>;
//template class MobileBaseController<OdometryFrame2WD,SkidSteeringKinematic>;
//template class MobileBaseController<OdometryFrame2FWS4WD,TwoWheelSteeringKinematic>;
//template class  MobileBaseController<OdometryFrame4WS4WD,FourWheelSteeringKinematic>;

}

//PLUGINLIB_EXPORT_CLASS(romea::MobileBaseController4WD, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(romea::MobileBaseController2WD, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(romea::MobileBaseController2FWS4WD, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(romea::MobileBaseController4WS4WD, controller_interface::ControllerBase);
