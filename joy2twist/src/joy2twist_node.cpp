#include "joy2twist/joy2twist_node.hpp"

namespace joy2twist
{
Joy2TwistNode::Joy2TwistNode(
  std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh)
: ph_(std::move(private_nh)), nh_(std::move(nh))
{
  load_parameters();

  if (e_stop_present_) {
    e_stop_sub_ = nh_->subscribe(e_stop_topic_, 1, &Joy2TwistNode::e_stop_cb, this);
    e_stop_reset_client_ = nh_->serviceClient<std_srvs::Trigger>(e_stop_reset_srv_);
    e_stop_trigger_client_ = nh_->serviceClient<std_srvs::Trigger>(e_stop_trigger_srv_);
  }
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

  ph_->param("e_stop/present", e_stop_present_, false);
  ph_->param("e_stop/topic", e_stop_topic_, std::string("e_stop"));
  ph_->param("e_stop/reset_srv", e_stop_reset_srv_, std::string("e_stop_reset"));
  ph_->param("e_stop/trigger_srv", e_stop_trigger_srv_, std::string("e_stop_trigger"));

  ph_->param("button_index_map/axis/angular_z", button_index_.angular_z, 2);
  ph_->param("button_index_map/axis/linear_x", button_index_.linear_x, 1);
  ph_->param("button_index_map/axis/linear_y", button_index_.linear_y, 0);
  ph_->param("button_index_map/dead_man_switch", button_index_.dead_man_switch, 4);
  ph_->param("button_index_map/fast_mode", button_index_.fast_mode, 7);
  ph_->param("button_index_map/slow_mode", button_index_.slow_mode, 5);
  ph_->param("button_index_map/e_stop_reset", button_index_.e_stop_reset, 1);
  ph_->param("button_index_map/e_stop_trigger", button_index_.e_stop_trigger, 2);
  ph_->param("button_index_map/enable_e_stop_reset", button_index_.enable_e_stop_reset, 6);
}

void Joy2TwistNode::e_stop_cb(const MsgBool & bool_msg) { e_stop_state_ = bool_msg.data; }

void Joy2TwistNode::joy_cb(const MsgJoy & joy_msg)
{
  MsgTwist twist_msg;

  if (e_stop_present_) {
    if (joy_msg.buttons.at(button_index_.e_stop_trigger) && !e_stop_state_) {
      twist_pub_.publish(twist_msg);
      call_trigger_service(e_stop_trigger_client_);
    } else if (
      joy_msg.buttons.at(button_index_.enable_e_stop_reset) &&
      joy_msg.buttons.at(button_index_.e_stop_reset) && e_stop_state_) {
      call_trigger_service(e_stop_reset_client_);
    }
  }

  if (joy_msg.buttons.at(button_index_.dead_man_switch)) {
    driving_mode_ = true;
    convert_joy_to_twist(joy_msg, twist_msg);
    twist_pub_.publish(twist_msg);
  } else if (driving_mode_) {
    driving_mode_ = false;
    twist_pub_.publish(twist_msg);
  }
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

void Joy2TwistNode::call_trigger_service(ros::ServiceClient & client)
{
  if (client.waitForExistence(ros::Duration(5.0))) {
    std_srvs::Trigger srv;
    client.call(srv);
    if (srv.response.success) {
      ROS_DEBUG(
        "[%s] Succesfully called %s service. Response: %s", ros::this_node::getName().c_str(),
        client.getService().c_str(), srv.response.message.c_str());
    } else {
      ROS_ERROR(
        "[%s] Failed to call %s service. Response: %s", ros::this_node::getName().c_str(),
        client.getService().c_str(), srv.response.message.c_str());
    }

  } else {
    ROS_ERROR(
      "[%s] Can't contact %s service", ros::this_node::getName().c_str(),
      client.getService().c_str());
  }
}

}  // namespace joy2twist
