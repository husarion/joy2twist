#include "joy2twist/joy2twist_node.hpp"

namespace joy2twist
{
Joy2TwistNode::Joy2TwistNode(
  std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh)
: ph_(std::move(private_nh)), nh_(std::move(nh))
{
  load_parameters();

  joy_sub_ = nh_->subscribe("joy", 1, &Joy2TwistNode::joy_cb, this);
  twist_pub_ = nh_->advertise<MsgTwist>("cmd_vel", 1);
}

void Joy2TwistNode::load_parameters()
{
  ph_->param("linear_velocity_factor/fast", linear_velocity_factors_[FAST], 1.0);
  ph_->param("linear_velocity_factor/regular", linear_velocity_factors_[REGULAR], 0.5);
  ph_->param("linear_velocity_factor/slow", linear_velocity_factors_[SLOW], 0.2);

  ph_->param("angular_velocity_factor/fast", angular_velocity_factors_[FAST], 1.0);
  ph_->param("angular_velocity_factor/regular", angular_velocity_factors_[REGULAR], 0.5);
  ph_->param("angular_velocity_factor/slow", angular_velocity_factors_[SLOW], 0.2);

  ph_->param("button_index_map/axis/angular_z", button_index_.angular_z, 0);
  ph_->param("button_index_map/axis/linear_x", button_index_.linear_x, 3);
  ph_->param("button_index_map/axis/linear_y", button_index_.linear_y, 2);
  ph_->param("button_index_map/dead_man_switch", button_index_.dead_man_switch, 4);
  ph_->param("button_index_map/fast_mode", button_index_.fast_mode, 7);
  ph_->param("button_index_map/slow_mode", button_index_.slow_mode, 5);
}

void Joy2TwistNode::joy_cb(const MsgJoy & joy_msg)
{
  MsgTwist twist_msg;
  if (joy_msg.buttons.at(button_index_.dead_man_switch)) {
    convert_joy_to_twist(joy_msg, twist_msg);
  }
  twist_pub_.publish(twist_msg);
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy & joy_msg, MsgTwist & twist_msg)
{
  float linear_velocity_factor{}, angular_velocity_factor{};
  std::tie(linear_velocity_factor, angular_velocity_factor) = determine_velocity_factor(joy_msg);

  twist_msg.angular.z = angular_velocity_factor * joy_msg.axes.at(button_index_.angular_z);
  twist_msg.linear.x = linear_velocity_factor * joy_msg.axes.at(button_index_.linear_x);
  twist_msg.linear.y = linear_velocity_factor * joy_msg.axes.at(button_index_.linear_y);
}

std::pair<float, float> Joy2TwistNode::determine_velocity_factor(const MsgJoy & joy_msg)
{
  auto linear_velocity_factor = linear_velocity_factors_.at(REGULAR);
  auto angular_velocity_factor = angular_velocity_factors_.at(REGULAR);

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
