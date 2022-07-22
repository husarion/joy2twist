#include "joy2twist/joy2twist_node.hpp"

namespace joy2twist
{
Joy2TwistNode::Joy2TwistNode(std::shared_ptr<ros::NodeHandle> nh) : nh_(std::move(nh))
{
  load_parameters();

  joy_sub_ = nh_->subscribe("/joy", 1, &Joy2TwistNode::joy_cb, this);
  twist_pub_ = nh_->advertise<MsgTwist>("/cmd_vel", 1);
}

void Joy2TwistNode::load_parameters()
{
  nh_->param(
    "/joy2twist_node/linear_velocity_factor", linear_velocity_factors_, defaults::VELOCITY_FACTORS);
  nh_->param(
    "/joy2twist_node/angular_velocity_factor", angular_velocity_factors_,
    defaults::VELOCITY_FACTORS);
  nh_->param(
    "/joy2twist_node/button_index_map/axis/angular_z", button_index_.angular_z,
    defaults::ANGULAR_Z);
  nh_->param(
    "/joy2twist_node/button_index_map/axis/linear_x", button_index_.linear_x, defaults::LINEAR_X);
  nh_->param(
    "/joy2twist_node/button_index_map/axis/linear_y", button_index_.linear_y, defaults::LINEAR_Y);
  nh_->param(
    "/joy2twist_node/button_index_map/dead_man_switch", button_index_.dead_man_switch,
    defaults::DEAD_MAN_SWITCH);
  nh_->param(
    "/joy2twist_node/button_index_map/fast_mode", button_index_.fast_mode, defaults::FAST_MODE);
  nh_->param(
    "/joy2twist_node/button_index_map/slow_mode", button_index_.slow_mode, defaults::SLOW_MODE);
}

void Joy2TwistNode::joy_cb(const MsgJoy & joy_msg)
{
  ROS_DEBUG("Processing callback for /joy topic");
  MsgTwist twist_msg;
  if (joy_msg.buttons.at(button_index_.dead_man_switch)) {
    convert_joy_to_twist(joy_msg, twist_msg);
  }
  twist_pub_.publish(twist_msg);
  ROS_DEBUG("Finished callback for /joy topic");
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy & joy_msg, MsgTwist & twist_msg)
{
  float linear_velocity_factor{}, angular_velocity_factor{};
  std::tie(linear_velocity_factor, angular_velocity_factor) = determine_speed_mode(joy_msg);

  twist_msg.angular.z = angular_velocity_factor * joy_msg.axes.at(button_index_.angular_z);
  twist_msg.linear.x = linear_velocity_factor * joy_msg.axes.at(button_index_.linear_x);
  twist_msg.linear.y = linear_velocity_factor * joy_msg.axes.at(button_index_.linear_y);
}

std::pair<float, float> Joy2TwistNode::determine_speed_mode(const MsgJoy & joy_msg)
{
  float linear_velocity_factor = linear_velocity_factors_.at(REGULAR);
  float angular_velocity_factor = angular_velocity_factors_.at(REGULAR);

  if (joy_msg.buttons.at(button_index_.slow_mode) && !joy_msg.buttons.at(button_index_.fast_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(SLOW);
    angular_velocity_factor = angular_velocity_factors_.at(SLOW);
  } else if (
    joy_msg.buttons.at(button_index_.fast_mode) && !joy_msg.buttons.at(button_index_.slow_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(FAST);
    angular_velocity_factor = angular_velocity_factors_.at(FAST);
  }
  return std::make_pair(linear_velocity_factor, angular_velocity_factor);
}

}  // namespace joy2twist
