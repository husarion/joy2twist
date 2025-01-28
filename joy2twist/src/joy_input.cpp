#include "joy2twist/joy_input.hpp"

namespace joy2twist
{
JoyInput JoyInput::from_string(const std::string & input)
{
  JoyInput joy_input;
  if (input.length() < 2) {
    throw std::invalid_argument("Invalid joy input string: " + input);
  }

  auto it = input.begin();

  if (*it == JoyInput::INVERTED_PREFIX) {
    joy_input.is_inverted = true;
    ++it;
  }

  // Determine the type of input
  switch (*it++) {
    case JoyInput::TYPE_AXIS:
      joy_input.type = JoyInput::Type::AXIS;
      break;

    case JoyInput::TYPE_BUTTON:
      joy_input.type = JoyInput::Type::BUTTON;
      break;

    default:
      throw std::invalid_argument("Invalid joy input string: " + input);
  }

  try {
    // take rest of the string from iterator and convert it to an integer
    joy_input.index = std::stoi(input.substr(it - input.begin()));
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("Invalid joy input string: " + input);
  }

  return joy_input;
}
}  // namespace joy2twist
