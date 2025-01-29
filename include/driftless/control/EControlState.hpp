#ifndef __E_CONTROL_STATE_HPP__
#define __E_CONTROL_STATE_HPP__

namespace driftless {
namespace control {
enum class EControlState {
  DRIVE_STRAIGHT_TARGET_REACHED,
  GO_TO_POINT_TARGET_REACHED,
  TURN_TARGET_REACHED,
  PATH_FOLLOWER_TARGET_REACHED
};
}  // namespace control
}  // namespace driftless
#endif