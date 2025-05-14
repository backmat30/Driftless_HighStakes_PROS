#ifndef __E_SUBSYSTEM_STATE_HPP__
#define __E_SUBSYSTEM_STATE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The enum class for subsystem states
/// @author Matthew Backman
enum class ESubsystemState {
  ARM_IS_NEUTRAL,
  ARM_IS_GOING_NEUTRAL,
  ARM_IS_LOAD,
  ARM_IS_GOING_LOAD,
  ARM_IS_READY,
  ARM_IS_GOING_READY,
  ARM_IS_SCORE,
  ARM_IS_GOING_SCORE,
  ARM_IS_RUSH,
  ARM_IS_GOING_RUSH,
  ARM_IS_ALLIANCE_STAKE,
  ARM_IS_GOING_ALLIANCE_STAKE,
  ARM_IS_CLIMB_READY,
  ARM_IS_GOING_CLIMB_READY,
  ARM_IS_CLIMB,
  CLAMP_GET_STATE,
  CLAMP_HAS_GOAL,
  CLIMB_ARE_PASSIVES_OUT,
  CLIMB_IS_CLIMBING,
  DRIVETRAIN_GET_VELOCITY,
  DRIVETRAIN_GET_RADIUS,
  DRIVETRAIN_GET_LEFT_POSITION,
  DRIVETRAIN_GET_RIGHT_POSITION,
  ELEVATOR_GET_POSITION,
  ELEVATOR_IS_DEPLOYED,
  ELEVATOR_IS_PAUSED,
  INTAKE_GET_HEIGHT,
  ODOMETRY_GET_POSITION,
  ODOMETRY_GET_RESETTER_RAW_VALUE,
  RING_SORT_HAS_RING,
  RING_SORT_GET_HUE,
  RING_SORT_GET_RGB,
  RING_SORT_GET_DISTANCE_TO_END
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif