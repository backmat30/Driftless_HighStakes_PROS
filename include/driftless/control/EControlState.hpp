#ifndef __E_CONTROL_STATE_HPP__
#define __E_CONTROL_STATE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Enumerated class for states of controls
/// @author Matthew Backman
enum class EControlState {
  DRIVE_STRAIGHT_TARGET_REACHED,
  GO_TO_POINT_TARGET_REACHED,
  TURN_TARGET_REACHED,
  PATH_FOLLOWER_TARGET_REACHED
};
}  // namespace control
}  // namespace driftless
#endif