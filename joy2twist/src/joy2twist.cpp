#include <iostream>

#include "joy2twist/ros/joy2twist_node.hpp"

using namespace joy2twist;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto joy2twist_node = std::make_shared<Joy2TwistNode>();
  executor.add_node(joy2twist_node);

  try {
    executor.spin();
  } catch (const std::runtime_error & err) {
    std::cerr << "[Joy2TwistNode] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[Joy2TwistNode] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
