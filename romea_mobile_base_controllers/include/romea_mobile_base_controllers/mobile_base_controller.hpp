#ifndef MobileBaseController_H
#define MobileBaseController_H

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>

//romea
#include "dead_reckoning.hpp"
#include "dead_reckoning_publisher.hpp"
#include "mobile_base_controller_traits.hpp"
#include <romea_common_utils/realtime_publishers/stamped_message_publisher.hpp>


namespace romea{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <typename OdometryFrameType, typename KinematicType>
class MobileBaseController : public controller_interface::ControllerInterface
{

  using Kinematic = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Kinematic;
  using MobileBaseInfo = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::MobileBaseInfo;
  using Command = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Command;
  using CommandMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::CommandMsg;
  using CommandRosMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::CommandRosMsg;
  using Constraints = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Constraints;
  using OdometryInterface = typename  MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryInterface;
  using OdometryMeasure = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryMeasure;
  using OdometryMeasureMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryMeasureMsg;
  using OdometryMeasurePublisher = RealtimeStampedMessagePublisher<OdometryMeasure,OdometryMeasureMsg>;
  using KinematicMeasurePublisher = RealtimeStampedMessagePublisher<KinematicMeasure,romea_mobile_base_msgs::msg::KinematicMeasureStamped>;

  struct StampedCommand
  {
    Command cmd;
    rclcpp::Time stamp;
  };

  using StampedCommandBuffer= realtime_tools::RealtimeBuffer<StampedCommand>;

public:

  MobileBaseController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:

  void declare_parameters_(std::shared_ptr<rclcpp::Node> node);
  void declare_mobile_base_info(std::shared_ptr<rclcpp::Node> node);
  void declare_command_constraints_(std::shared_ptr<rclcpp::Node> node);
  void declare_publish_period_(std::shared_ptr<rclcpp::Node> node);
  void declare_command_timeout_(std::shared_ptr<rclcpp::Node> node);

  void get_mobile_base_info(std::shared_ptr<rclcpp::Node> node);
  void get_command_constraints_(std::shared_ptr<rclcpp::Node> node);
  void get_publish_period_(std::shared_ptr<rclcpp::Node> node);
  void get_command_timeout_(std::shared_ptr<rclcpp::Node> node);

  void init_publishers_(std::shared_ptr<rclcpp::Node> node);
  void init_cmd_subscriber_();
  void init_interface_(std::shared_ptr<rclcpp::Node> node);


  void reset_();
  void brake_();
  bool timeout_();
  void update_controller_state_();
  void publish_controller_state_();
  bool new_command_available_();
  void clamp_current_command_();
  void send_current_command_();
  void command_callback_(const typename CommandMsg::ConstPtr & cmd_msg);

protected:

  std::unique_ptr<OdometryInterface> odometry_interface_;
  typename Kinematic::Parameters kinematic_parameters_;
  Constraints user_command_contraints_;
  OdometryMeasure odometry_measure_;
  KinematicMeasure kinematic_measure_;
  OdometryFrameType odometry_frame_;

  typename rclcpp::Subscription<CommandMsg>::SharedPtr command_sub;
  StampedCommand previous_command_;
  StampedCommand current_command_;
  StampedCommandBuffer command_buffer_;

  rclcpp::Time update_time_;
  rclcpp::Time last_state_publish_time_;
  rclcpp::Duration publish_period_;
  rclcpp::Duration command_timeout_;

  DeadReckoningPublisher dead_reckoning_publisher_;
  OdometryMeasurePublisher odometry_measure_publisher_;
  KinematicMeasurePublisher kinematic_measure_publisher_;

  //  struct WheelHandle
  //  {
  //    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
  //    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  //  };

  //  const char * feedback_type() const;
  //  CallbackReturn configure_side(
  //    const std::string & side, const std::vector<std::string> & wheel_names,
  //    std::vector<WheelHandle> & registered_handles);

  //  std::vector<std::string> left_wheel_names_;
  //  std::vector<std::string> right_wheel_names_;

  //  std::vector<WheelHandle> registered_left_wheel_handles_;
  //  std::vector<WheelHandle> registered_right_wheel_handles_;

  //  struct WheelParams
  //  {
  //    size_t wheels_per_side = 0;
  //    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
  //    double radius = 0.0;      // Assumed to be the same for both wheels
  //    double separation_multiplier = 1.0;
  //    double left_radius_multiplier = 1.0;
  //    double right_radius_multiplier = 1.0;
  //  } wheel_params_;

  //  struct OdometryParams
  //  {
  //    bool open_loop = false;
  //    bool position_feedback = true;
  //    bool enable_odom_tf = true;
  //    std::string base_frame_id = "base_link";
  //    std::string odom_frame_id = "odom";
  //    std::array<double, 6> pose_covariance_diagonal;
  //    std::array<double, 6> twist_covariance_diagonal;
  //  } odom_params_;

  //  Odometry odometry_;

  //  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  //  realtime_odometry_publisher_ = nullptr;

  //  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
  //      nullptr;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
  //  realtime_odometry_transform_publisher_ = nullptr;

  //  // Timeout to consider cmd_vel commands old
  //  std::chrono::milliseconds cmd_vel_timeout_{500};

  //  bool subscriber_is_active_ = false;
  //  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  //  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
  //  velocity_command_unstamped_subscriber_ = nullptr;

  //  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  //  std::queue<Twist> previous_commands_;  // last two commands

  //  // speed limiters
  //  SpeedLimiter limiter_linear_;
  //  SpeedLimiter limiter_angular_;

  //  bool publish_limited_velocity_ = false;
  //  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
  //      nullptr;

  //  rclcpp::Time previous_update_timestamp_{0};

  //  // publish rate limiter
  //  double publish_rate_ = 50.0;
  //  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  //  rclcpp::Time previous_publish_timestamp_{0};

  //  bool is_halted = false;
  //  bool use_stamped_vel_ = true;

  //  bool reset();
  //  void halt();
};


//template <typename OdometryFrameType, typename KinematicType>
//class MobileBaseController :
//        public controller_interface::MultiInterfaceController<
//                    hardware_interface::VelocityJointInterface,
//                    hardware_interface::PositionJointInterface>
//        //        public controller_interface::Controller<hardware_interface::VelocityJointInterface>
//{

//private :

//    using Kinematic = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Kinematic;
//    using Command = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Command;
//    using CommandMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::CommandMsg;
//    using CommandRosMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::CommandRosMsg;
//    using Constraints = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::Constraints;
//    using OdometryInterface = typename  MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryInterface;
//    using OdometryMeasure = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryMeasure;
//    using OdometryMeasureMsg = typename MobileBaseControllerTraits<OdometryFrameType,KinematicType>::OdometryMeasureMsg;
//    using OdometryMeasurePublisher = RealtimeStampedMessagePublisher<OdometryMeasure,OdometryMeasureMsg>;
//    using KinematicMeasurePublisher = RealtimeStampedMessagePublisher<KinematicMeasure,romea_mobile_base_msgs::KinematicMeasureStamped>;

//    struct StampedCommand
//    {
//        Command cmd;
//        ros::Time stamp;
//    };

//    using StampedCommandBuffer= realtime_tools::RealtimeBuffer<StampedCommand>;

//public:

//    MobileBaseController();

//    virtual ~MobileBaseController()=default;

//    /**
//     * \brief Initialize controller
//     * \param hw            Velocity joint interface for the wheels
//     * \param root_nh       Node handle at root namespace
//     * \param controller_nh Node handle inside the controller namespace
//     */
//    virtual bool init(hardware_interface::RobotHW *hw,
//                      ros::NodeHandle& root_nh,
//                      ros::NodeHandle &controller_nh)override;

//    /**
//     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
//     * \param time   Current time
//     * \param period Time since the last called to update
//     */
//    virtual void update(const ros::Time& time, const ros::Duration& period) override;

//    /**
//     * \brief Starts controller
//     * \param time Current time
//     */
//    virtual void starting(const ros::Time& time) override;

//    /**
//     * \brief Stops controller
//     * \param time Current time
//     */
//    virtual void stopping(const ros::Time& /*time*/);


//protected :

//    std::string name_;

//    std::unique_ptr<OdometryInterface> odometry_interface_;
//    typename Kinematic::Parameters kinematic_parameters_;
//    Constraints user_command_contraints_;
//    OdometryMeasure odometry_measure_;
//    KinematicMeasure kinematic_measure_;
//    OdometryFrameType odometry_frame_;

//    ros::Subscriber command_sub;
//    StampedCommand previous_command_;
//    StampedCommand current_command_;
//    StampedCommandBuffer command_buffer_;

//    ros::Time update_time_;
//    ros::Time last_state_publish_time_;
//    ros::Duration publish_period_;
//    ros::Duration command_timeout_;

//    DeadReckoningPublisher dead_reckoning_publisher_;
//    OdometryMeasurePublisher odometry_measure_publisher_;
//    KinematicMeasurePublisher kinematic_measure_publisher_;


//protected :

//    void loadKinematicParams_(ros::NodeHandle & root_nh);
//    void loadControllerName_(ros::NodeHandle & controller_nh);
//    void loadCommandConstraints_(ros::NodeHandle & controller_nh);
//    void loadPublishPeriod_(ros::NodeHandle & controller_nh);
//    void loadCommandTimeout_(ros::NodeHandle & controller_nh);
//    void initPublishers_(ros::NodeHandle & controller_nh);
//    void initCmdSubscriber_(ros::NodeHandle & controller_nh);

//    void initInterface_(hardware_interface::RobotHW *hw,
//                        ros::NodeHandle & root_nh,
//                        ros::NodeHandle & controller_nh);

//    void brake_();
//    bool timeout_();
//    void update_controller_state_();
//    void publish_controller_state_();
//    bool new_command_available_();
//    void clamp_current_command_();
//    void send_current_command_();
//    void command_callback_(const typename CommandMsg::ConstPtr & cmd_msg);
//};

//using MobileBaseController4WD =  MobileBaseController<OdometryFrame4WD,SkidSteeringKinematic>;
//using MobileBaseController2WD =  MobileBaseController<OdometryFrame2WD,SkidSteeringKinematic>;
//using MobileBaseController2FWS4WD =  MobileBaseController<OdometryFrame2FWS4WD,TwoWheelSteeringKinematic>;
//using MobileBaseController4WS4WD =  MobileBaseController<OdometryFrame4WS4WD,FourWheelSteeringKinematic>;

}

#endif
