//#ifndef _romea_CommandListener_hpp_
//#define _romea_CommandListener_hpp_

////romea
//#include "../conversions/command_conversions.hpp"
//#include <romea_common_utils/listeners/data_listener.hpp>
//#include <romea_common_utils/qos.hpp>

//namespace romea {

//template<class CommandType>
//class CommandListener
//{

//public:

//  using ListenerBase = DataListenerBase<CommandType>;
//  using ListenerBasePtr = std::unique_ptr<ListenerBase>;


//public :

//  CommandListener(std::shared_ptr<rclcpp::Node> node,
//                  const std::string & message_type);

//  CommandType  get_command() const;

//  std::string get_topic_name() const;

//private :

//  template <class MessageType>
//  ListenerBasePtr make_listener_(std::shared_ptr<rclcpp::Node> node,
//                                 const std::string & topic_name,
//                                 const size_t &queue_size)
//  {
//    using ListenerType = DataListener<CommandType,MessageType,rclcpp::Node>;
//    return std::make_unique<ListenerType>(node,topic_name,best_effort(1));
//  }

//  ListenerBasePtr make_listener_(std::shared_ptr<rclcpp::Node> node,
//                                 const std::string & message_type);

//private :

//  ListenerBasePtr listener_;

//};


//}
//#endif
