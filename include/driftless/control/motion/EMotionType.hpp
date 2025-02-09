#ifndef __E_MOTION_TYPE_HPP__
#define __E_MOTION_TYPE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for basic motion control algorithms
/// @author Matthew Backman
namespace motion {

/// @brief Enumeration for the possible motion control types
/// @author Matthew Backman
enum class EMotionType { NONE, DRIVE_STRAIGHT, GO_TO_POINT, TURN };
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif