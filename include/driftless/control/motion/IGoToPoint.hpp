#ifndef __I_GO_TO_POINT_HPP__
#define __I_GO_TO_POINT_HPP__

#include <memory>

#include "driftless/control/Point.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for control algorithms
namespace control {
/// @brief Namespace for basic motion control algorithms
namespace motion {
/// @brief Interface for a generic __go to point__ algorithm
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
  /// @param robot - The robot being controlled
  /// @param velocity - The maximum velocity of the robot
  /// @param point - The point for the robot to go to
  virtual void goToPoint(const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
                         Point point) = 0;

  /// @brief Sets the maximum velocity during motion
  /// @param velocity - The new maximum velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Determines if the robot has reached the desired point
  /// @return __TRUE__ if the robot reached the desired point, __FALSE__ if not
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif