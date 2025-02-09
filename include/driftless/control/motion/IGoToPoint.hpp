#ifndef __I_GO_TO_POINT_HPP__
#define __I_GO_TO_POINT_HPP__

#include <memory>

#include "driftless/control/Point.hpp"
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

/// @brief Interface for a generic go to point algorithm
/// @author Matthew Backman
class IGoToPoint {
 public:
  /// @brief Destroys the go to point object
  virtual ~IGoToPoint() = default;

  /// @brief Initializes the go to point algorithm
  virtual void init() = 0;

  /// @brief Runs the go to point algorithm
  virtual void run() = 0;

  /// @brief Pauses the go to point algorithm
  virtual void pause() = 0;

  /// @brief Resumes the go to point algorithm
  virtual void resume() = 0;

  /// @brief Drives the given robot to a point
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param velocity __double__ The maximum velocity of the robot
  /// @param point __Point__ The point for the robot to go to
  virtual void goToPoint(const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
                         Point point) = 0;

  /// @brief Sets the maximum velocity during motion
  /// @param velocity __double__ The new maximum velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Determines if the robot has reached the desired point
  /// @return __bool__ True if the robot reached the desired point, else false
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif