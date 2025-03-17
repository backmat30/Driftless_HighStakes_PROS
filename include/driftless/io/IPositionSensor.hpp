#ifndef __I_POSITION_SENSOR_HPP__
#define __I_POSITION_SENSOR_HPP__

#include "driftless/robot/subsystems/odometry/Position.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic position tracker
class IPositionSensor {
 public:
  /// @brief Destroys the position tracker
  virtual ~IPositionSensor() = default;

  /// @brief Initializes the position tracker
  virtual void init() = 0;

  /// @brief Gets the position from the position tracker
  /// @return __robot::subsystems::odometry::Position__ The position
  virtual robot::subsystems::odometry::Position getPosition() = 0;

  /// @brief Sets the local offset of the position tracker
  /// @param x_offset __double__ The local x-offset
  /// @param y_offset __double__ The local y-offset
  /// @param theta_offset __double The local theta-offset
  virtual void setLocalOffset(double x_offset, double y_offset, double theta_offset) = 0;
};
}  // namespace io
}  // namespace driftless
#endif