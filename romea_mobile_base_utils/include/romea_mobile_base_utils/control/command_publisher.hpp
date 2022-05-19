#ifndef _romea_CommandPublisher_hpp_
#define _romea_CommandPublisher_hpp_

//romea
#include "../conversions/command_conversions.hpp"
#include <romea_common_utils/publishers/message_publisher.hpp>

namespace romea {



template<class CommandType>
class CommandPublisher
{

public:

  using PublisherBase = MessagePublisherBase<CommandType>;
  using PublisherBasePtr = std::unique_ptr<MessagePublisherBase<CommandType>>;


public :

  CommandPublisher(std::shared_ptr<rclcpp::Node> node,
                   const std::string & output_message_type);

  void publish(const CommandType & command);

  const std::string & get_topic_name() const;

private :

  template <class MessageType>
  PublisherBasePtr make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & topic_name,
                                   const size_t &queue_size)
  {
    using PublisherType = MessagePublisher<CommandType,MessageType>;
    return std::make_unique<PublisherType>(node,topic_name,queue_size);
  }

  PublisherBasePtr make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & topic_name);

private :

  std::unique_ptr<MessagePublisherBase<CommandType>> publisher_;

};


}
#endif
