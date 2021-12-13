
#include <cstdio>

#include "ROS1NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros1
{

ROS1Config ROS1NodeConfig::get_ros1_config() const
{
  ROS1Config ros1_config;
  ros1_config.dds_domain = dds_domain;
  ros1_config.dds_ros1_to_ros2_topic = dds_ros1_to_ros2_topic;
  ros1_config.dds_ros2_to_ros1_topic = dds_ros2_to_ros1_topic;
  return ros1_config;
}

ROS1NodeConfig ROS1NodeConfig::make()
{
  ROS1NodeConfig config;
  ros::NodeHandle node_private_ns("~");
  return config;
}

} // namespace ros1
} // namespace cyclone_bridge
