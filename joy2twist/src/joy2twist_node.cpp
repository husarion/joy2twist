#include "joy2twist/joy2twist_node.hpp"

namespace joy2twist
{
using std::placeholders::_1;

Joy2TwistNode::Joy2TwistNode() : Node("joy2twist_node")
{
  using namespace std::placeholders;

  declare_parameters();
  load_parameters();

  joy_sub_ = create_subscription<MsgJoy>(
    "joy", rclcpp::SensorDataQoS(), std::bind(&Joy2TwistNode::joy_cb, this, _1));
  twist_pub_ = create_publisher<MsgTwist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());

  if (e_stop_present_) {
    e_stop_sub_ = this->create_subscription<MsgBool>(
      e_stop_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&Joy2TwistNode::e_stop_cb, this, _1));
    e_stop_reset_client_ = this->create_client<SrvTrigger>(e_stop_reset_srv_);
    e_stop_trigger_client_ = this->create_client<SrvTrigger>(e_stop_trigger_srv_);
  }

  RCLCPP_INFO(get_logger(), "Initialized node!");
}

void Joy2TwistNode::declare_parameters()
{
  this->declare_parameter<float>("linear_velocity_factor.fast", 1.0);
  this->declare_parameter<float>("linear_velocity_factor.regular", 0.5);
  this->declare_parameter<float>("linear_velocity_factor.slow", 0.2);
  this->declare_parameter<float>("angular_velocity_factor.fast", 1.0);
  this->declare_parameter<float>("angular_velocity_factor.regular", 0.5);
  this->declare_parameter<float>("angular_velocity_factor.slow", 0.2);

  this->declare_parameter<bool>("e_stop.present", false);
  this->declare_parameter<std::string>("e_stop.topic", "e_stop");
  this->declare_parameter<std::string>("e_stop.reset_srv", "e_stop_reset");
  this->declare_parameter<std::string>("e_stop.trigger_srv", "e_stop_trigger");

  this->declare_parameter<int>("button_index_map.axis.angular_z", 2);
  this->declare_parameter<int>("button_index_map.axis.linear_x", 1);
  this->declare_parameter<int>("button_index_map.axis.linear_y", 0);
  this->declare_parameter<int>("button_index_map.dead_man_switch", 4);
  this->declare_parameter<int>("button_index_map.fast_mode", 7);
  this->declare_parameter<int>("button_index_map.slow_mode", 5);
  this->declare_parameter<int>("button_index_map.e_stop_reset", 1);
  this->declare_parameter<int>("button_index_map.e_stop_trigger", 2);
  this->declare_parameter<int>("button_index_map.enable_e_stop_reset", 6);
}

void Joy2TwistNode::load_parameters()
{
  this->get_parameter<float>("linear_velocity_factor.fast", linear_velocity_factors_[FAST]);
  this->get_parameter<float>("linear_velocity_factor.regular", linear_velocity_factors_[REGULAR]);
  this->get_parameter<float>("linear_velocity_factor.slow", linear_velocity_factors_[SLOW]);
  this->get_parameter<float>("angular_velocity_factor.fast", angular_velocity_factors_[FAST]);
  this->get_parameter<float>("angular_velocity_factor.regular", angular_velocity_factors_[REGULAR]);
  this->get_parameter<float>("angular_velocity_factor.slow", angular_velocity_factors_[SLOW]);

  this->get_parameter<bool>("e_stop.present", e_stop_present_);
  this->get_parameter<std::string>("e_stop.topic", e_stop_topic_);
  this->get_parameter<std::string>("e_stop.reset_srv", e_stop_reset_srv_);
  this->get_parameter<std::string>("e_stop.trigger_srv", e_stop_trigger_srv_);

  this->get_parameter<int>("button_index_map.axis.angular_z", button_index_.angular_z);
  this->get_parameter<int>("button_index_map.axis.linear_x", button_index_.linear_x);
  this->get_parameter<int>("button_index_map.axis.linear_y", button_index_.linear_y);
  this->get_parameter<int>("button_index_map.dead_man_switch", button_index_.dead_man_switch);
  this->get_parameter<int>("button_index_map.fast_mode", button_index_.fast_mode);
  this->get_parameter<int>("button_index_map.slow_mode", button_index_.slow_mode);
  this->get_parameter<int>("button_index_map.e_stop_reset", button_index_.e_stop_reset);
  this->get_parameter<int>("button_index_map.e_stop_trigger", button_index_.e_stop_trigger);
  this->get_parameter<int>(
    "button_index_map.enable_e_stop_reset", button_index_.enable_e_stop_reset);
}

void Joy2TwistNode::e_stop_cb(const MsgBool::SharedPtr bool_msg) { e_stop_state_ = bool_msg->data; }

void Joy2TwistNode::joy_cb(const MsgJoy::SharedPtr joy_msg)
{
  MsgTwist twist_msg;

  handle_e_stop(joy_msg);

  if (joy_msg->buttons.at(button_index_.dead_man_switch)) {
    driving_mode_ = true;
    convert_joy_to_twist(joy_msg, twist_msg);
    twist_pub_->publish(twist_msg);
  } else if (driving_mode_) {
    driving_mode_ = false;
    twist_pub_->publish(twist_msg);
  }
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

void Joy2TwistNode::call_trigger_service(const rclcpp::Client<SrvTrigger>::SharedPtr & client) const
{
  if (!client->wait_for_service(std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(this->get_logger(), "Can't contact %s service", client->get_service_name());
    return;
  }

  const auto request = std::make_shared<SrvTrigger::Request>();

  client->async_send_request(request, [&](const rclcpp::Client<SrvTrigger>::SharedFuture future) {
    trigger_service_cb(future, client->get_service_name());
  });
}

void Joy2TwistNode::trigger_service_cb(
  const rclcpp::Client<SrvTrigger>::SharedFuture & future, const std::string & service_name) const
{
  if (!future.get()->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to call %s service: %s", service_name.c_str(),
      future.get()->message.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully called %s service", service_name.c_str());
}

void Joy2TwistNode::handle_e_stop(const std::shared_ptr<MsgJoy> joy_msg)
{
  if (!e_stop_present_) {
    return;
  }

  if (joy_msg->buttons.at(button_index_.e_stop_trigger)) {
    if (!e_stop_state_) {
      // Stop the robot before trying to call the e-stop trigger service
      twist_pub_->publish(MsgTwist());
      call_trigger_service(e_stop_trigger_client_);
    }
    return;
  }

  if (
    joy_msg->buttons.at(button_index_.enable_e_stop_reset) &&
    joy_msg->buttons.at(button_index_.e_stop_reset) && e_stop_state_) {
    call_trigger_service(e_stop_reset_client_);
  }
}

}  // namespace joy2twist
