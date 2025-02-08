#ifndef __I_PATH_FOLLOWER_HPP__
#define __I_PATH_FOLLOWER_HPP__

#include <memory>
#include <vector>

#include "driftless/control/Point.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Interface for a generic path follower
/// @author Matthew Backman
class IPathFollower {
 public:
  /// @brief Destroys the path follower
  virtual ~IPathFollower() = default;

  /// @brief Initializes the path follower
  virtual void init() = 0;

  /// @brief Runs the path follower
  virtual void run() = 0;

  /// @brief Pauses the path follower
  virtual void pause() = 0;

  /// @brief Resumes the path follower
  virtual void resume() = 0;

  /// @brief Follows a given path
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param control_path __const std::vector<Point>&__ The list of control
  /// points
  /// @param velocity __double__ The maximum velocity
  virtual void followPath(const std::shared_ptr<driftless::robot::Robot>& robot,
                          const std::vector<Point>& control_path,
                          double velocity) = 0;

  /// @brief Sets the max velocity of the path follower
  /// @param velocity __double__ The new max velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Determines if the target position has been reached
  /// @return __bool__ True if within target range, false otherwise
  virtual bool targetReached() = 0;
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif