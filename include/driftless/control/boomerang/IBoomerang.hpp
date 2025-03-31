#ifndef __I_BOOMERANG_HPP__
#define __I_BOOMERANG_HPP__

#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the boomerang control
/// @author Matthew Backman
namespace boomerang {

/// @brief Interface for a generic boomerang control
/// @author Matthew Backman
class IBoomerang {
 public:
  /// @brief Destroys the boomerang control
  virtual ~IBoomerang() = default;

  /// @brief Initializes the boomerang control
  virtual void init() = 0;

  /// @brief Runs the boomerang control
  virtual void run() = 0;

  /// @brief Goes to a given position
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The maximum velocity
  /// @param target_x __double__ The target x position
  /// @param target_y __double__ The target y position
  /// @param target_theta __double__ The target theta position
  virtual void goToPosition(const std::shared_ptr<robot::Robot>& robot,
                            double velocity, double target_x, double target_y,
                            double target_theta) = 0;

  /// @brief Pauses the boomerang control
  virtual void pause() = 0;

  /// @brief Resumes the boomerang control
  virtual void resume() = 0;

  /// @brief Determines if the target position has been reached
  /// @return __bool__ True if the target position has been reached, false
  /// otherwise
  virtual bool targetReached() = 0;
};
}  // namespace boomerang
}  // namespace control
}  // namespace driftless
#endif