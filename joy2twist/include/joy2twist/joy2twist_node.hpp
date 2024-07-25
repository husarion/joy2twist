#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP_
#define JOY2TWIST_JOY2TWIST_NODE_HPP_

#include <map>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace joy2twist
{
using MsgJoy = sensor_msgs::msg::Joy;
using MsgTwist = geometry_msgs::msg::Twist;
using MsgBool = std_msgs::msg::Bool;
using SrvTrigger = std_srvs::srv::Trigger;

struct ButtonIndex
{
  int angular_z;
  int linear_x;
  int linear_y;

  int dead_man_switch;
  int fast_mode;
  int slow_mode;

  int e_stop_reset;
  int e_stop_trigger;
  int enable_e_stop_reset;
};

class Joy2TwistNode : public rclcpp::Node
{
public:
  Joy2TwistNode();

private:
  void declare_parameters();
  void load_parameters();

  void e_stop_cb(const std::shared_ptr<MsgBool> bool_msg);
  void joy_cb(const std::shared_ptr<MsgJoy> joy_msg);
  void convert_joy_to_twist(const std::shared_ptr<MsgJoy> joy_msg, MsgTwist & twist_msg);
  std::pair<float, float> determine_velocity_factor(const std::shared_ptr<MsgJoy> joy_msg);
  void call_trigger_service(const rclcpp::Client<SrvTrigger>::SharedPtr & client) const;
  void trigger_service_cb(
    const rclcpp::Client<SrvTrigger>::SharedFuture & future,
    const std::string & service_name) const;
  void handle_e_stop(const std::shared_ptr<MsgJoy> joy_msg);

  std::map<std::string, float> linear_velocity_factors_;
  std::map<std::string, float> angular_velocity_factors_;

  ButtonIndex button_index_;
  bool driving_mode_;
  bool e_stop_present_;
  bool e_stop_state_;
  std::string e_stop_topic_;
  std::string e_stop_reset_srv_;
  std::string e_stop_trigger_srv_;

  rclcpp::Subscription<MsgBool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<MsgJoy>::SharedPtr joy_sub_;
  rclcpp::Publisher<MsgTwist>::SharedPtr twist_pub_;
  rclcpp::Client<SrvTrigger>::SharedPtr e_stop_reset_client_;
  rclcpp::Client<SrvTrigger>::SharedPtr e_stop_trigger_client_;
};

static constexpr char FAST[]{"fast"};
static constexpr char REGULAR[]{"regular"};
static constexpr char SLOW[]{"slow"};
}  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP_
