#ifndef __E_CONTROLLER_ANALOG_HPP__
#define __E_CONTROLLER_ANALOG_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Enumerated class for analog controller inputs
/// @author Matthew Backman
enum class EControllerAnalog {
  JOYSTICK_LEFT_X,
  JOYSTICK_LEFT_Y,
  JOYSTICK_RIGHT_X,
  JOYSTICK_RIGHT_Y,
  NONE
};
}  // namespace op_control
}  // namespace driftless
#endif