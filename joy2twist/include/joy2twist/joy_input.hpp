#pragma once

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

  Type type = Type::AXIS;
  int index = -1;
  bool is_inverted = false;

  static constexpr char TYPE_AXIS = 'A';
  static constexpr char TYPE_BUTTON = 'B';
  static constexpr char INVERTED_PREFIX = '!';

  static JoyInput from_string(const std::string & input);
};
}  // namespace joy2twist
