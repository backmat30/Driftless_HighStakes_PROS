#ifndef __E_CONTROL_TYPE_HPP__
#define __E_CONTROL_TYPE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Enumerated class for controllable objects
/// @author Matthew Backman
enum EControlType { ARM, CLAMP, CLIMB, DRIVE, ELEVATOR, INTAKE, COLOR_SORT };
}  // namespace op_control
}  // namespace driftless
#endif