#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP_
#define JOY2TWIST_JOY2TWIST_NODE_HPP_

#include <map>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy2twist
{
using MsgJoy = sensor_msgs::msg::Joy;
using MsgTwist = geometry_msgs::msg::Twist;

struct ButtonIndex
{
  int angular_z;
  int linear_x;
  int linear_y;

  int dead_man_switch;
  int fast_mode;
  int slow_mode;
};

class Joy2TwistNode : public rclcpp::Node
{
public:
  Joy2TwistNode();

private:
  void declare_parameters();
  void load_parameters();

  void joy_cb(const std::shared_ptr<MsgJoy> joy_msg);
  void convert_joy_to_twist(const std::shared_ptr<MsgJoy> joy_msg, MsgTwist & twist_msg);
  std::pair<float, float> determine_velocity_factor(const std::shared_ptr<MsgJoy> joy_msg);

  std::map<std::string, float> linear_velocity_factors_;
  std::map<std::string, float> angular_velocity_factors_;

  ButtonIndex button_index_;
  bool driving_mode_;

  rclcpp::Subscription<MsgJoy>::SharedPtr joy_sub_;
  rclcpp::Publisher<MsgTwist>::SharedPtr twist_pub_;
};

static constexpr char FAST[]{"fast"};
static constexpr char REGULAR[]{"regular"};
static constexpr char SLOW[]{"slow"};
}  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP_
