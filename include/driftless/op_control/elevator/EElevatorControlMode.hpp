#ifndef __E_ELEVATOR_CONTROL_MODE_HPP__
#define __E_ELEVATOR_CONTROL_MODE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for elevator control
/// @author Matthew Backman
namespace elevator {

/// @brief Enumerated class representing potential elevator control modes
/// @author Matthew Backman
enum class EElevatorControlMode { TOGGLE, HOLD };
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless
#endif