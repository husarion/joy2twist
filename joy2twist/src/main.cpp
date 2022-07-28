#include <iostream>

#include "joy2twist/joy2twist_node.hpp"

using namespace joy2twist;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "joy2twist_node");

  auto private_nh = std::make_shared<ros::NodeHandle>("~");
  auto nh = std::make_shared<ros::NodeHandle>();
  Joy2TwistNode joy2twist_node(private_nh, nh);

  ROS_INFO_STREAM(
    "[" << ros::this_node::getName() << "]"
        << " Initialized node!");

  try {
    ros::spin();
  } catch (const std::exception & e) {
    std::cerr << "[" << ros::this_node::getName() << "]"
              << " Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[" << ros::this_node::getName() << "]"
            << " Shutting down" << std::endl;
  ros::shutdown();
  return 0;
}
