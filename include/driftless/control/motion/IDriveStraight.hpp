#ifndef __I_DRIVE_STRAIGHT_HPP__
#define __I_DRIVE_STRAIGHT_HPP__

#include <memory>

#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for control algorithms
namespace control {
/// @brief Namespace for basic motion control algorithms
namespace motion {
/// @brief interface for a generic __drive straight__ algorithm
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
  /// @param robot - The robot being controlled
  /// @param velocity - The max speed of the robot during motion
  /// @param distance - The distance to travel
  /// @param theta - The angle to the target
  virtual void driveStraight(std::shared_ptr<driftless::robot::Robot>& robot,
                             double velocity, double distance,
                             double theta) = 0;

  /// @brief Sets the maximum velocity during motion
  /// @param velocity - The new maximum velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Determines if the robot is at the desired position
  /// @return __TRUE__ if the robot reached the target, __FALSE__ if not
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif