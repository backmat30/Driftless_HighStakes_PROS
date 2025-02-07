#ifndef __E_ARM_CONTROL_MODE_HPP__
#define __E_ARM_CONTROL_MODE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for arm control
/// @author Matthew Backman
namespace arm {

/// @brief Enumerated class for arm control modes
/// @author Matthew Backman
enum class EArmControlMode { SPLIT_TOGGLE, SMART_TOGGLE };
}  // namespace arm
}  // namespace op_control
}  // namespace driftless
#endif