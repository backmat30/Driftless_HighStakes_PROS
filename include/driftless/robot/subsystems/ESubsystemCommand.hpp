#ifndef __E_SUBSYSTEM_COMMAND_HPP__
#define __E_SUBSYSTEM_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {
  
/// @brief The enum class for subsystem commands
/// @author Matthew Backman
enum class ESubsystemCommand {
  ARM_GO_NEUTRAL,
  ARM_GO_LOAD,
  ARM_GO_READY,
  ARM_GO_SCORE,
  ARM_GO_RUSH,
  ARM_GO_ALLIANCE_STAKE,
  ARM_GO_PREVIOUS,
  ARM_CALIBRATE,
  CLAMP_SET_STATE,
  CLIMB_TOGGLE_CLIMBING,
  CLIMB_PULL_BACK_CLIMBER,
  CLIMB_PUSH_FORWARD_CLIMBER,
  DRIVETRAIN_SET_VELOCITY,
  DRIVETRAIN_SET_VOLTAGE,
  ELEVATOR_SET_VOLTAGE,
  ELEVATOR_SET_POSITION,
  ELEVATOR_DEPLOY_REJECTOR,
  ELEVATOR_RETRACT_REJECTOR,
  INTAKE_SET_VOLTAGE,
  INTAKE_SET_HEIGHT,
  ODOMETRY_SET_POSITION,
  ODOMETRY_SET_X,
  ODOMETRY_SET_Y,
  ODOMETRY_SET_THETA,
  ODOMETRY_RESET_X,
  ODOMETRY_RESET_Y

};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif