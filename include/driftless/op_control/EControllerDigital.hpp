#ifndef __E_CONTROLLER_DIGITAL_HPP__
#define __E_CONTROLLER_DIGITAL_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Enumerated class for digital controller inputs
/// @author Matthew Backman
enum class EControllerDigital {
  BUTTON_A,
  BUTTON_B,
  BUTTON_X,
  BUTTON_Y,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_UP,
  DPAD_RIGHT,
  TRIGGER_LEFT_TOP,
  TRIGGER_LEFT_BOTTOM,
  TRIGGER_RIGHT_TOP,
  TRIGGER_RIGHT_BOTTOM,
  NONE
};
}  // namespace op_control
}  // namespace driftless
#endif