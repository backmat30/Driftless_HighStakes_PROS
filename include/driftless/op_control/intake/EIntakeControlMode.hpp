#ifndef __E_INTAKE_CONTROL_MODE_HPP__
#define __E_INTAKE_CONTROL_MODE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for intake control
/// @author Matthew Backman
namespace intake {

/// @brief Enumerated class representing potential intake control modes
enum class EIntakeControlMode { SPLIT_TOGGLE, SINGLE_TOGGLE, HOLD_UP };
}  // namespace intake
}  // namespace op_control
}  // namespace driftless
#endif