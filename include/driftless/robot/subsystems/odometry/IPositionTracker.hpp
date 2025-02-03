#ifndef __I_POSITION_TRACKER_HPP__
#define __I_POSITION_TRACKER_HPP__

#include "driftless/robot/subsystems/odometry/Position.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Interface for position trackers
/// @author Matthew Backman
class IPositionTracker {
 public:
  /// @brief Destructor
  virtual ~IPositionTracker() = default;

  /// @brief Initializes the position tracker
  virtual void init() = 0;

  /// @brief Runs the position tracker
  virtual void run() = 0;

  /// @brief Sets the position of the tracking system
  /// @param position __Position__ The position to set
  virtual void setPosition(Position position) = 0;

  /// @brief Gets the position of the robot
  /// @return __Position__ The current position of the robot
  virtual Position getPosition() = 0;

  /// @brief Sets the x position
  /// @param x __double__ The x position to set
  virtual void setX(double x) = 0;

  /// @brief Sets the y position
  /// @param y __double__ The y position to set
  virtual void setY(double y) = 0;

  /// @brief Sets the angle
  /// @param theta __double__ The angle to set
  virtual void setTheta(double theta) = 0;
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif