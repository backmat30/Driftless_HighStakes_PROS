#ifndef __E_SUBSYSTEM_HPP__
#define __E_SUBSYSTEM_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The enum class for subsystems
/// @author Matthew Backman
enum class ESubsystem {
  ARM,
  CLAMP,
  DRIVETRAIN,
  ELEVATOR,
  INTAKE,
  ODOMETRY,
  RING_SORT
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif