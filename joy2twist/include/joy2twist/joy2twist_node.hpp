#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP_
#define JOY2TWIST_JOY2TWIST_NODE_HPP_

#include <map>
#include <utility>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

namespace joy2twist
{
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
};

class Joy2TwistNode
{
public:
  Joy2TwistNode(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh);

private:
  std::shared_ptr<ros::NodeHandle> ph_;
  std::shared_ptr<ros::NodeHandle> nh_;

  void load_parameters();

  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;

  void joy_cb(const MsgJoy & joy_msg);
  void convert_joy_to_twist(const MsgJoy & joy_msg, MsgTwist & twist_msg);
  std::pair<float, float> determine_velocity_factor(const MsgJoy & joy_msg);

  std::map<std::string, double> linear_velocity_factors_;
  std::map<std::string, double> angular_velocity_factors_;

  ButtonIndex button_index_;
};

static constexpr char FAST[]{"fast"};
static constexpr char REGULAR[]{"regular"};
static constexpr char SLOW[]{"slow"};

};  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP_
