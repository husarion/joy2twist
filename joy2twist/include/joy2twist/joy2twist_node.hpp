#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP
#define JOY2TWIST_JOY2TWIST_NODE_HPP

#include <string.h>

#include <map>
#include <utility>

#include <ros/console.h>
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
  Joy2TwistNode(std::shared_ptr<ros::NodeHandle> nh);

private:
  std::shared_ptr<ros::NodeHandle> nh_;

  void load_parameters();

  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;

  void joy_cb(const MsgJoy & joy_msg);
  void convert_joy_to_twist(const MsgJoy & joy_msg, MsgTwist & twist_msg);
  std::pair<float, float> determine_velocity_factor(const MsgJoy & joy_msg);

  std::map<std::string, float> linear_velocity_factors_;
  std::map<std::string, float> angular_velocity_factors_;

  ButtonIndex button_index_;
};

namespace defaults
{
  static constexpr int DEAD_MAN_SWITCH{4};
  static constexpr int FAST_MODE{7};
  static constexpr int SLOW_MODE{5};

  static constexpr int ANGULAR_Z{0};
  static constexpr int LINEAR_X{3};
  static constexpr int LINEAR_Y{2};

  const std::map<std::string, float> VELOCITY_FACTORS{
    {"fast", 1.0}, {"regular", 0.5}, {"slow", 0.2}};
}  // namespace defaults

static constexpr char FAST[]{"fast"};
static constexpr char REGULAR[]{"regular"};
static constexpr char SLOW[]{"slow"};

};  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP
