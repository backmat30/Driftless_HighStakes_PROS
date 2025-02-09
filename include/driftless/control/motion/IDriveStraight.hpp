#ifndef __I_DRIVE_STRAIGHT_HPP__
#define __I_DRIVE_STRAIGHT_HPP__

#include <memory>

#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for basic motion control algorithms
/// @author Matthew Backman
namespace motion {

/// @brief Interface for a generic drive straight algorithm
/// @author Matthew Backman
class IDriveStraight {
 public:
  /// @brief Destroys the drive straight object
  virtual ~IDriveStraight() = default;

  /// @brief Initializes the drive straight algorithm
  virtual void init() = 0;

  /// @brief Runs the drive straight algorithm
  virtual void run() = 0;

  /// @brief Pauses the drive straight algorithm
  virtual void pause() = 0;

  /// @brief Resumes the drive straight algorithm
  virtual void resume() = 0;

  /// @brief Drives the given robot forwards
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max speed of the robot during motion
  /// @param distance __double__ The distance to travel
  /// @param theta __double__ The angle to the target
  virtual void driveStraight(std::shared_ptr<driftless::robot::Robot>& robot,
                             double velocity, double distance,
                             double theta) = 0;

  /// @brief Sets the maximum velocity during motion
  /// @param velocity __double__ The new maximum velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Determines if the robot is at the desired position
  /// @return __bool__ True if the robot reached the target, else false
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif