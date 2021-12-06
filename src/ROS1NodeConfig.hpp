
#ifndef CYCLONE_ROS1_NODE__SRC__ROS1NODECONFIG_HPP
#define CYCLONE_ROS1_NODE__SRC__ROS1NODECONFIG_HPP

#include <string>

#include <ros/ros.h>

#include <cyclone_bridge/ROS1Config.hpp>

namespace cyclone_bridge
{
namespace ros1
{

struct ROS1NodeConfig
{
  std::string request_topic = "/request";
  std::string response_topic = "/response";

  int dds_domain = 100;

  std::string dds_request_topic = "request";
  std::string dds_response_topic = "response";

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key, 
      std::string& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      int& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      double& param_out);
      
  void print_config() const;

  ROS1Config get_ros1_config() const;

  static ROS1NodeConfig make();

};

} // namespace ros1
} // namespace cyclone_bridge

#endif // CYCLONE_ROS1_NODE__SRC__ROS1NODECONFIG_HPP
