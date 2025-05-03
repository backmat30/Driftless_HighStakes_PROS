#ifndef __OP_CONTROL_E_CONTROL_HPP__
#define __OP_CONTROL_E_CONTROL_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Enumerated class for operator control commands
/// @author Matthew Backman
enum EControl {
  ARM_NEUTRAL,
  ARM_LOAD,
  ARM_READY,
  ARM_SCORE,
  ARM_RUSH,
  ARM_TOGGLE,
  ARM_CLIMB_CYCLE,
  ARM_CALIBRATE,
  ARM_ALLIANCE_STAKE,
  CLAMP_GRAB,
  CLAMP_RELEASE,
  CLAMP_HOLD,
  CLAMP_TOGGLE,
  CLIMB_CHANGE_HEIGHT,
  CLIMB_TOGGLE,
  CLIMB_RELEASE,
  CLIMB_TOGGLE_PASSIVES,
  COLOR_SORT_TOGGLE,
  DRIVE_ARCADE_LINEAR,
  DRIVE_ARCADE_TURN,
  ELEVATOR_SPIN,
  ELEVATOR_REVERSE,
  ELEVATOR_TOGGLE,
  INTAKE_SPIN,
  INTAKE_REVERSE,
  INTAKE_RAISE,
  INTAKE_LOWER,
  INTAKE_TOGGLE_HEIGHT,
  INTAKE_HOLD_UP
};
}  // namespace op_control
}  // namespace driftless
#endif