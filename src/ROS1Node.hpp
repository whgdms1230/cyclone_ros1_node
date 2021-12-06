
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

// #include "cyclone_ros1_node/Operator.h"
#include "cyclone_ros1_node/Request.h"
#include "cyclone_ros1_node/Response.h"
#include "cyclone_ros1_node/Result.h"
#include "cyclone_ros1_node/Variable.h"

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

  void print_config();

private:

  std::unique_ptr<ros::NodeHandle> node;

  std::unique_ptr<ros::Rate> update_rate;

  std::unique_ptr<ros::Rate> publish_rate;

  ros::Subscriber request_sub;

  void request_callback_fn(const cyclone_ros1_node::Request& _msg);

  bool read_response();

  void send_request();

  ROS1NodeConfig ros1_node_config;

  Fields fields;

  ROS1Node(const ROS1NodeConfig& config);

  void start(Fields fields);

  std::string problem_name;

};

} // namespace ros1
} // namespace cyclone_bridge

#endif // CYCLONE_ROS1_NODE__SRC__ROS1NODE_HPP