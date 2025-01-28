#ifndef JOY2TWIST_JOY_INPUT_HPP_
#define JOY2TWIST_JOY_INPUT_HPP_

#include <stdexcept>
#include <string>

namespace joy2twist
{

class JoyInput
{
public:
  enum class Type {
    AXIS,
    BUTTON,
  };

  static constexpr char kTypeAxis = 'A';
  static constexpr char kTypeButton = 'B';
  static constexpr char kInvertedPrefix = '!';

  Type type = Type::AXIS;
  int index = -1;
  bool is_inverted = false;

  static JoyInput from_string(const std::string & input);
};
}  // namespace joy2twist

#endif  // JOY2TWIST_JOY_INPUT_HPP_
