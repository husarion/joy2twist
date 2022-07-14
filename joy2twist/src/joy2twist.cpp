#include <iostream>
#include <joy2twist/ros/joy2twist_node.hpp>

using namespace joy2twist;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "joy2twist_node");

  auto nh = std::make_shared<ros::NodeHandle>();
  Joy2TwistNode joy2twist_node(nh);

  ROS_INFO("Initialized joy2twist node!");

  try {
    ros::spin();
  } catch (const std::exception &e) {
    std::cerr << "[Joy2TwistNode] Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[Joy2TwistNode] Shutting down" << std::endl;
  ros::shutdown();
  return 0;
}
