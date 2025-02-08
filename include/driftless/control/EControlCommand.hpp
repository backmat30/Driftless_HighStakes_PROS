#ifndef __E_CONTROL_COMMAND_HPP__
#define __E_CONTROL_COMMAND_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Enumerated class representing commands for controls
/// @author Matthew Backman
enum class EControlCommand {
  DRIVE_STRAIGHT,
  GO_TO_POINT,
  TURN_TO_ANGLE,
  TURN_TO_POINT,
  FOLLOW_PATH,
  DRIVE_STRAIGHT_SET_VELOCITY,
  GO_TO_POINT_SET_VELOCITY,
  TURN_SET_VELOCITY,
  PATH_FOLLOWER_SET_VELOCITY
};
}  // namespace control
}  // namespace driftless
#endif