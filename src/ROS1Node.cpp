
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
{
  if (read_thread.joinable())
  {
    read_thread.join();
  }
}

void ROS1Node::start(Fields _fields)
{
  fields = std::move(_fields);

  read_rate.reset(new ros::Rate(ros1_node_config.read_frequency));

  send_topic_sub = node->subscribe(
      ros1_node_config.ros1_to_ros2_topic, 1, &ROS1Node::send_topic_cb, this);

  read_topic_pub = node->advertise<cyclone_ros1_node::IntNumber>(ros1_node_config.ros2_to_ros1_topic, 10);

  read_thread = std::thread(std::bind(&ROS1Node::read_thread_fn, this));
}

void ROS1Node::send_topic_cb(
    const cyclone_ros1_node::IntNumber& _msg)
{
  new_number = _msg.int_num;

  send();
}

void ROS1Node::send()
{
  messages::IntNumber ros1_to_ros2_num;
  ros1_to_ros2_num.int_num = new_number;

  fields.ros1_bridge->send(ros1_to_ros2_num);
}

void ROS1Node::read()
{
  messages::IntNumber ros2_to_ros1_num;
  if (fields.ros1_bridge->read(ros2_to_ros1_num))
  {
    cyclone_ros1_node::IntNumber new_num;
    new_num.int_num = ros2_to_ros1_num.int_num;

    read_topic_pub.publish(new_num);
  }
}

void ROS1Node::read_thread_fn()
{
  while (node->ok())
  {
    read_rate->sleep();
    
    // read message from DDS
    read();
  }
}

} // namespace ros1
} // namespace cyclone_bridge
