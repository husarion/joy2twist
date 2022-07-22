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
  this->declare_parameter<float>(
    "linear_velocity_factor.fast", defaults::VELOCITY_FACTORS.at(FAST));
  this->declare_parameter<float>(
    "linear_velocity_factor.regular", defaults::VELOCITY_FACTORS.at(REGULAR));
  this->declare_parameter<float>(
    "linear_velocity_factor.slow", defaults::VELOCITY_FACTORS.at(SLOW));
  this->declare_parameter<float>(
    "angular_velocity_factor.fast", defaults::VELOCITY_FACTORS.at(FAST));
  this->declare_parameter<float>(
    "angular_velocity_factor.regular", defaults::VELOCITY_FACTORS.at(REGULAR));
  this->declare_parameter<float>(
    "angular_velocity_factor.slow", defaults::VELOCITY_FACTORS.at(SLOW));
  this->declare_parameter<int>("button_index_map.axis.angular_z", defaults::ANGULAR_Z);
  this->declare_parameter<int>("button_index_map.axis.linear_x", defaults::LINEAR_X);
  this->declare_parameter<int>("button_index_map.axis.linear_y", defaults::LINEAR_Y);
  this->declare_parameter<int>("button_index_map.dead_man_switch", defaults::DEAD_MAN_SWITCH);
  this->declare_parameter<int>("button_index_map.fast_mode", defaults::FAST_MODE);
  this->declare_parameter<int>("button_index_map.slow_mode", defaults::SLOW_MODE);
}

void Joy2TwistNode::load_parameters()
{
  this->get_parameter<float>("linear_velocity_factor.fast", linear_velocity_factors_[FAST]);
  this->get_parameter<float>("linear_velocity_factor.regular", linear_velocity_factors_[REGULAR]);
  this->get_parameter<float>("linear_velocity_factor.slow", linear_velocity_factors_[SLOW]);
  this->get_parameter<float>("angular_velocity_factor.fast", angular_velocity_factors_[FAST]);
  this->get_parameter<float>("angular_velocity_factor.regular", angular_velocity_factors_[REGULAR]);
  this->get_parameter<float>("angular_velocity_factor.slow", angular_velocity_factors_[SLOW]);
  this->get_parameter<int>("button_index_map.axis.angular_z", button_index_.angular_z);
  this->get_parameter<int>("button_index_map.axis.linear_x", button_index_.linear_x);
  this->get_parameter<int>("button_index_map.axis.linear_y", button_index_.linear_y);
  this->get_parameter<int>("button_index_map.dead_man_switch", button_index_.dead_man_switch);
  this->get_parameter<int>("button_index_map.fast_mode", button_index_.fast_mode);
  this->get_parameter<int>("button_index_map.slow_mode", button_index_.slow_mode);
}

void Joy2TwistNode::joy_cb(const MsgJoy::SharedPtr joy_msg)
{
  RCLCPP_DEBUG(get_logger(), "Processing callback for /joy topic");
  MsgTwist twist_msg;
  if (joy_msg->buttons.at(button_index_.dead_man_switch)) {
    convert_joy_to_twist(joy_msg, twist_msg);
  }
  twist_pub_->publish(twist_msg);
  RCLCPP_DEBUG(get_logger(), "Finished callback for /joy topic");
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy::SharedPtr joy_msg, MsgTwist & twist_msg)
{
  float linear_velocity_factor{}, angular_velocity_factor{};
  std::tie(linear_velocity_factor, angular_velocity_factor) = determine_velocity_factor(joy_msg);

  twist_msg.angular.z = angular_velocity_factor * joy_msg->axes.at(button_index_.angular_z);
  twist_msg.linear.x = linear_velocity_factor * joy_msg->axes.at(button_index_.linear_x);
  twist_msg.linear.y = linear_velocity_factor * joy_msg->axes.at(button_index_.linear_y);
}

std::pair<float, float> Joy2TwistNode::determine_velocity_factor(const MsgJoy::SharedPtr joy_msg)
{
  float linear_velocity_factor = linear_velocity_factors_.at(REGULAR);
  float angular_velocity_factor = angular_velocity_factors_.at(REGULAR);
  if (
    joy_msg->buttons.at(button_index_.slow_mode) && !joy_msg->buttons.at(button_index_.fast_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(SLOW);
    angular_velocity_factor = angular_velocity_factors_.at(SLOW);
  } else if (
    joy_msg->buttons.at(button_index_.fast_mode) && !joy_msg->buttons.at(button_index_.slow_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(FAST);
    angular_velocity_factor = angular_velocity_factors_.at(FAST);
  }
  return std::make_pair(linear_velocity_factor, angular_velocity_factor);
}

}  // namespace joy2twist
