#ifndef __E_MOTION_TYPE_HPP__
#define __E_MOTION_TYPE_HPP__

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for control algorithms
namespace control {
/// @brief Namespace for basic motion control algorithms
namespace motion {
/// @brief Enumeration for the possible motion control types
enum class EMotionType { NONE, DRIVE_STRAIGHT, GO_TO_POINT, TURN };
}
}  // namespace control
}  // namespace driftless
#endif