
#include <cstdio>

#include "ROS1NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros1
{

void ROS1NodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    std::string& _param_out)
{
  std::string tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %s", 
        _key.c_str(), _key.c_str(), tmp_param.c_str());
    _param_out = tmp_param;
  }
}

void ROS1NodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    int& _param_out)
{
  int tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %d.",
        _key.c_str(), _key.c_str(), tmp_param);
    _param_out = tmp_param;
  }
}

void ROS1NodeConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    double& _param_out)
{
  double tmp_param;
  if (_node.getParam(_key, tmp_param))
  {
    ROS_INFO("Found %s on the parameter server. Setting %s to %.2f.",
        _key.c_str(), _key.c_str(), tmp_param);
    _param_out = tmp_param;
  }
}

void ROS1NodeConfig::print_config() const
{
  printf("ROS 1 NODE CONFIGURATION\n");
  printf("  TOPICS\n");
  printf("    request topic: %s\n", request_topic.c_str());
  printf("    response topic: %s\n", response_topic.c_str());
  printf("ROS1-ROS2 DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    request topic: %s\n", dds_request_topic.c_str());
  printf("    response topic: %s\n", dds_response_topic.c_str());
}
  
ROS1Config ROS1NodeConfig::get_ros1_config() const
{
  ROS1Config ros1_config;
  ros1_config.dds_domain = dds_domain;
  ros1_config.dds_request_topic = dds_request_topic;
  ros1_config.dds_response_topic = dds_response_topic;
  return ros1_config;
}

ROS1NodeConfig ROS1NodeConfig::make()
{
  ROS1NodeConfig config;
  ros::NodeHandle node_private_ns("~");

  config.get_param_if_available(
      node_private_ns, "request_topic", config.request_topic);
  config.get_param_if_available(
      node_private_ns, "response_topic", config.response_topic);
  config.get_param_if_available(
      node_private_ns, "dds_request_topic", 
      config.dds_request_topic);
  config.get_param_if_available(
      node_private_ns, "dds_response_topic", 
      config.dds_response_topic);

  return config;
}

} // namespace ros1
} // namespace cyclone_bridge
