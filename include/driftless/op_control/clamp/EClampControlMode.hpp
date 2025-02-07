#ifndef __E_CLAMP_CONTROL_MODE_HPP__
#define __E_CLAMP_CONTROL_MODE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for clamp control
/// @author Matthew Backman
namespace clamp {

/// @brief Enumerated class representing potential clamp control modes
/// @author Matthew Backman
enum class EClampControlMode { SINGLE_TOGGLE, SPLIT_TOGGLE, HOLD };
}
}  // namespace op_control
}  // namespace driftless
#endif