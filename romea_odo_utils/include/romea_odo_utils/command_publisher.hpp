#ifndef _romea_CommandPublisher_hpp_
#define _romea_CommandPublisher_hpp_

//romea
#include "command_conversions.hpp"
#include <romea_common_utils/publishers/message_publisher.hpp>

namespace romea {


template<class CommandType>
class CommandPublisher
{

public :

  CommandPublisher();

  void init(std::shared_ptr<rclcpp::Node> node,
            const std::string & controller_type);

  void publish(const CommandType & command);

  //  const std::string & getTopic() const;

private :


  template <class MessageType>
  void make_publisher_(std::shared_ptr<rclcpp::Node> node,
                       const std::string & topic_name,
                       const size_t &queue_size)
  {
    publisher_ = std::make_unique<MessagePublisher<CommandType,MessageType>>(node,topic_name,queue_size);
  }

private :

  std::unique_ptr<MessagePublisherBase<CommandType>> publisher_;
  //  std::string publisher_topic_;

};


}
#endif
