#ifndef __I_TURN_HPP__
#define __I_TURN_HPP__

#include <memory>

#include "driftless/control/Point.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
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

/// @brief Interface for a generic turn algorithm
/// @author Matthew Backman
class ITurn {
 public:
  /// @brief Destroys the turn object
  virtual ~ITurn() = default;

  /// @brief Initializes the turn algorithm
  virtual void init() = 0;

  /// @brief Runs the turn algorithm
  virtual void run() = 0;

  /// @brief Pauses the turn algorithm
  virtual void pause() = 0;

  /// @brief Resumes the turn algorithm
  virtual void resume() = 0;

  /// @brief Turns the robot to face a given angle
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max velocity during motion
  /// @param theta __double__ The angle to turn towards
  /// @param direction __ETurnDirection__ The direction to turn in, defaults to
  /// __AUTO__
  virtual void turnToAngle(
      const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
      double theta, ETurnDirection direction = ETurnDirection::AUTO) = 0;

  /// @brief Turns the robot to face a given point
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max velocity during motion
  /// @param point __Point__ The point to turn towards
  /// @param direction __ETurnDirection__ The direction to turn in, defaults to
  /// __AUTO__
  virtual void turnToPoint(
      const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
      Point point, ETurnDirection direction = ETurnDirection::AUTO) = 0;

  /// @brief Determines if the robot has reached the target rotation
  /// @return __bool__ True if the robot has reached the target, else false
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif