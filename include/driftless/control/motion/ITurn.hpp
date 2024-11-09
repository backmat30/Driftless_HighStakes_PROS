#ifndef __I_TURN_HPP__
#define __I_TURN_HPP__

#include <memory>

#include "driftless/control/Point.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for control algorithms
namespace control {
/// @brief Namespace for basic motion control algorithms
namespace motion {
/// @brief Interface for a generic __turn__ algorithm
class ITurn {
 public:
  /// @brief Destroys the turn object
  virtual ~ITurn() = default;

  /// @brief Initialize the turn algorithm
  virtual void init() = 0;

  /// @brief Runs the turn algorithm
  virtual void run() = 0;

  /// @brief Pauses the turn algorithm
  virtual void pause() = 0;

  /// @brief Resumes the turn algorithm
  virtual void resume() = 0;

  /// @brief Turn the robot to face a given angle
  /// @param robot - The robot being controlled
  /// @param velocity - The max velocity during motion
  /// @param theta - The angle to turn towards
  /// @param direction - The direction to turn in, defaults to __AUTO__
  virtual void turnToAngle(
      const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
      double theta, ETurnDirection direction = ETurnDirection::AUTO) = 0;

  /// @brief Turn the robot to face a given point
  /// @param robot - The robot being controlled
  /// @param velocity - The max velocity during motion
  /// @param point - The point to turn towards
  /// @param direction - The direction to turn in, defaults to __AUTO__
  virtual void turnToPoint(
      const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
      Point point, ETurnDirection direction = ETurnDirection::AUTO) = 0;

  /// @brief Determines if the robot has reached the target rotation
  /// @return __TRUE__ if the robot has reached the target, __FALSE__ if not
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif