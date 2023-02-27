#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP_
#define JOY2TWIST_JOY2TWIST_NODE_HPP_

#include <map>
#include <utility>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

namespace joy2twist
{
using MsgBool = std_msgs::Bool;
using MsgJoy = sensor_msgs::Joy;
using MsgTwist = geometry_msgs::Twist;

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

class Joy2TwistNode
{
public:
  Joy2TwistNode(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh);

private:
  std::shared_ptr<ros::NodeHandle> ph_;
  std::shared_ptr<ros::NodeHandle> nh_;

  void load_parameters();

  ros::Subscriber e_stop_sub_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::ServiceClient e_stop_reset_client_;
  ros::ServiceClient e_stop_trigger_client_;

  void e_stop_cb(const MsgBool & bool_msg);
  void joy_cb(const MsgJoy & joy_msg);
  void convert_joy_to_twist(const MsgJoy & joy_msg, MsgTwist & twist_msg);
  void call_trigger_service(ros::ServiceClient & client);
  std::pair<float, float> determine_velocity_factor(const MsgJoy & joy_msg);

  std::map<std::string, double> linear_velocity_factors_;
  std::map<std::string, double> angular_velocity_factors_;

  ButtonIndex button_index_;
  bool driving_mode_;
  bool e_stop_present_;
  bool e_stop_state_;
  std::string e_stop_topic_;
  std::string e_stop_reset_srv_;
  std::string e_stop_trigger_srv_;
};

static constexpr char FAST[]{"fast"};
static constexpr char REGULAR[]{"regular"};
static constexpr char SLOW[]{"slow"};

};  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP_
