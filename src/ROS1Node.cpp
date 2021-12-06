
#include "ROS1Node.hpp"
#include "ROS1NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros1
{

ROS1Node::SharedPtr ROS1Node::make(const ROS1NodeConfig& _config)
{
  SharedPtr ros1_node = SharedPtr(new ROS1Node(_config));
  ros1_node->node.reset(new ros::NodeHandle("ros1_node"));

  /// Starting the ros1 node
  ROS1Config ros1_config = _config.get_ros1_config();
  ROS1Bridge::SharedPtr ros1_bridge = ROS1Bridge::make(ros1_config);
  if (!ros1_bridge)
    return nullptr;

  ros1_node->start(Fields{
      std::move(ros1_bridge)
  });

  return ros1_node;
}

ROS1Node::ROS1Node(const ROS1NodeConfig& _config) :
  ros1_node_config(_config)
{}

ROS1Node::~ROS1Node()
{}

void ROS1Node::start(Fields _fields)
{
  fields = std::move(_fields);
}

void ROS1Node::print_config()
{
  ros1_node_config.print_config();
}

void ROS1Node::request_callback_fn(
    const cyclone_ros1_node::Request& _msg)
{
  problem_name = _msg.name;

  send_request();
}

void ROS1Node::send_request()
{
  messages::Request new_request;
  new_request.name = problem_name;

  fields.ros1_bridge->send_request(new_request);
}

bool ROS1Node::read_response()
{
  messages::Response new_response;
  if (fields.ros1_bridge->read_response(new_response))
  {
    return true;
  }
  return false;
}

} // namespace ros1
} // namespace cyclone_bridge
