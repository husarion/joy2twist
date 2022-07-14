#include "joy2twist/ros/joy2twist_node.hpp"

namespace joy2twist {

Joy2TwistNode::Joy2TwistNode(std::shared_ptr<ros::NodeHandle> nh)
    : nh_(std::move(nh)) {

  load_parameters();

  joy_sub_ = nh_->subscribe("/joy", 1, &Joy2TwistNode::joy_cb, this);
  twist_pub_ = nh_->advertise<MsgTwist>("/cmd_vel", 1);
}

void Joy2TwistNode::load_parameters() {
  if (!nh_->param("/joy2twist_node/velocity_factor", velocity_factors_,
                  default_velocity_factors_)) {
    ROS_WARN("Couldn't load parameters, using default values!");
  }
}

void Joy2TwistNode::joy_cb(const MsgJoy &joy_msg) {
  MsgTwist twist_msg;

  if (joy_msg.buttons.at(DEAD_MAN_SWITCH)) {
    convert_joy_to_twist(joy_msg, twist_msg);
  }

  twist_pub_.publish(twist_msg);
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy &joy_msg,
                                         MsgTwist &twist_msg) {
  auto velocity_factor = determine_speed_mode(joy_msg);

  twist_msg.angular.z =
      velocity_factor * angular_velocity_factor * joy_msg.axes.at(ANGULAR_Z);
  twist_msg.linear.x = velocity_factor * joy_msg.axes.at(LINEAR_X);
  twist_msg.linear.y = velocity_factor * joy_msg.axes.at(LINEAR_Y);
}

float Joy2TwistNode::determine_speed_mode(const MsgJoy &joy_msg) {
  float velocity_factor = velocity_factors_.at(REGULAR);
  if (joy_msg.buttons.at(SLOW_MODE) && !joy_msg.buttons.at(FAST_MODE)) {
    velocity_factor = velocity_factors_.at(SLOW);
  } else if (joy_msg.buttons.at(FAST_MODE) && !joy_msg.buttons.at(SLOW_MODE)) {
    velocity_factor = velocity_factors_.at(FAST);
  }
  return velocity_factor;
}

} // namespace joy2twist
