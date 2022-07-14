#ifndef JOY2TWIST_NODE_HTTP
#define JOY2TWIST_NODE

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy2twist {
using MsgJoy = sensor_msgs::msg::Joy;
using MsgTwist = geometry_msgs::msg::Twist;

class Joy2TwistNode : public rclcpp::Node {
public:
  Joy2TwistNode();

private:
  void declare_parameters();
  void load_parameters();

  void joy_cb(const std::shared_ptr<MsgJoy> joy_msg);

  void convert_joy_to_twist(const std::shared_ptr<MsgJoy> joy_msg,
                            MsgTwist &twist_msg);

  float determine_speed_mode(const std::shared_ptr<MsgJoy> joy_msg);

  std::map<std::string, float> velocity_factors_;

  const std::map<std::string, float> default_velocity_factors_{
      {"fast", 1.0}, {"regular", 0.25}, {"slow", 0.1}};

  static constexpr int angular_velocity_factor = 2;

  rclcpp::Subscription<MsgJoy>::SharedPtr joy_sub_;
  rclcpp::Publisher<MsgTwist>::SharedPtr twist_pub_;
};

static constexpr int DEAD_MAN_SWITCH = 4;
static constexpr int FAST_MODE = 7;
static constexpr int SLOW_MODE = 5;
static constexpr int ANGULAR_Z = 0;
static constexpr int LINEAR_X = 3;
static constexpr int LINEAR_Y = 2;

static constexpr char FAST[] = "fast";
static constexpr char REGULAR[] = "regular";
static constexpr char SLOW[] = "slow";
} // namespace joy2twist

#endif // JOY2TWIST_NODE
