
#include "ROS1NodeConfig.hpp"
#include "ROS1Node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyclone_ros1_node");
  ros::NodeHandle ros_node_handle;
  ROS_INFO("Greetings from cyclone_ros1_node");

  auto config = cyclone_bridge::ros1::ROS1NodeConfig::make();

  auto ros1_node = cyclone_bridge::ros1::ROS1Node::make(config);

  if (!ros1_node)
  {
    ROS_ERROR("cyclone_ros1_node: unable to initialize.");
    return 1;
  }

  ros::spin();
  
  return 0;
}
