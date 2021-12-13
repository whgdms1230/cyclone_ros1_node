
#ifndef CYCLONE_ROS1_NODE__SRC__ROS1NODE_HPP
#define CYCLONE_ROS1_NODE__SRC__ROS1NODE_HPP

#include <deque>
#include <mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <cyclone_bridge/ROS1Bridge.hpp>

#include "cyclone_ros1_node/IntNumber.h"

#include "ROS1NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros1
{

class ROS1Node
{
public:

  using SharedPtr = std::shared_ptr<ROS1Node>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(const ROS1NodeConfig& ros1_config);

  ~ROS1Node();

  struct Fields
  {
    /// ros1 bridge client
    ROS1Bridge::SharedPtr ros1_bridge;

  };

private:

  std::unique_ptr<ros::Rate> read_rate;

  std::unique_ptr<ros::NodeHandle> node;

  ros::Subscriber send_topic_sub;

  ros::Publisher read_topic_pub;

  void send_topic_cb(const cyclone_ros1_node::IntNumber& _msg);

  void read();

  void send();

  ROS1NodeConfig ros1_node_config;

  Fields fields;

  ROS1Node(const ROS1NodeConfig& config);

  void start(Fields fields);

  uint32_t new_number;

  std::thread read_thread;

  void read_thread_fn();
};

} // namespace ros1
} // namespace cyclone_bridge

#endif // CYCLONE_ROS1_NODE__SRC__ROS1NODE_HPP
