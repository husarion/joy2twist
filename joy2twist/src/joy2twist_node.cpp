#include "joy2twist/joy2twist_node.hpp"
namespace joy2twist
{
Joy2TwistNode::Joy2TwistNode() : Node("joy2twist_node")
{
  using namespace std::placeholders;

  declare_parameters();
  load_parameters();

  auto sensor_qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth),
    rmw_qos_profile_sensor_data);

  joy_sub_ =
    create_subscription<MsgJoy>("/joy", sensor_qos, std::bind(&Joy2TwistNode::joy_cb, this, _1));
  twist_pub_ = create_publisher<MsgTwist>("/cmd_vel", sensor_qos);

  RCLCPP_INFO(get_logger(), "Joy2Twist node is initialized");
}

void Joy2TwistNode::declare_parameters()
{
  this->declare_parameter<float>("velocity_factor.fast", default_velocity_factors_.at(FAST));
  this->declare_parameter<float>("velocity_factor.regular", default_velocity_factors_.at(REGULAR));
  this->declare_parameter<float>("velocity_factor.slow", default_velocity_factors_.at(SLOW));
}

void Joy2TwistNode::load_parameters()
{
  this->get_parameter<float>("velocity_factor.fast", velocity_factors_[FAST]);
  this->get_parameter<float>("velocity_factor.regular", velocity_factors_[REGULAR]);
  this->get_parameter<float>("velocity_factor.slow", velocity_factors_[SLOW]);
}

void Joy2TwistNode::joy_cb(const MsgJoy::SharedPtr joy_msg)
{
  RCLCPP_DEBUG(get_logger(), "Processing callback for /joy topic");
  MsgTwist twist_msg;
  if (joy_msg->buttons.at(DEAD_MAN_SWITCH)) {
    convert_joy_to_twist(joy_msg, twist_msg);
  }
  twist_pub_->publish(twist_msg);
  RCLCPP_DEBUG(get_logger(), "Finished callback for /joy topic");
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy::SharedPtr joy_msg, MsgTwist & twist_msg)
{
  auto velocity_factor = determine_speed_mode(joy_msg);

  twist_msg.angular.z = velocity_factor * angular_velocity_factor * joy_msg->axes.at(ANGULAR_Z);
  twist_msg.linear.x = velocity_factor * joy_msg->axes.at(LINEAR_X);
  twist_msg.linear.y = velocity_factor * joy_msg->axes.at(LINEAR_Y);
}

float Joy2TwistNode::determine_speed_mode(const MsgJoy::SharedPtr joy_msg)
{
  float velocity_factor = velocity_factors_.at(REGULAR);
  if (joy_msg->buttons.at(SLOW_MODE) && !joy_msg->buttons.at(FAST_MODE)) {
    velocity_factor = velocity_factors_.at(SLOW);
  } else if (joy_msg->buttons.at(FAST_MODE) && !joy_msg->buttons.at(SLOW_MODE)) {
    velocity_factor = velocity_factors_.at(FAST);
  }
  return velocity_factor;
}

}  // namespace joy2twist
