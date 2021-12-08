
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
  std::string ros1_to_ros2_topic = "/ros1_to_ros2_topic";
  std::string ros2_to_ros1_topic = "/ros2_to_ros1_topic";

  int dds_domain = 100;

  std::string dds_ros1_to_ros2_topic = "ros1_to_ros2";
  std::string dds_ros2_to_ros1_topic = "ros2_to_ros1";
      
  ROS1Config get_ros1_config() const;

  static ROS1NodeConfig make();

};

} // namespace ros1
} // namespace cyclone_bridge

#endif // CYCLONE_ROS1_NODE__SRC__ROS1NODECONFIG_HPP
