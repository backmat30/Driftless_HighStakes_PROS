#ifndef __I_ROTATION_SENSOR_HPP__
#define __I_ROTATION_SENSOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic rotation sensor
/// @author Matthew Backman
class IRotationSensor {
 public:
  /// @brief Destroys the current rotation sensor object
  virtual ~IRotationSensor() = default;

  /// @brief Initializes the rotation sensor
  virtual void init() = 0;

  /// @brief Resets the rotation sensor
  virtual void reset() = 0;

  /// @brief Gets the rotations from the rotation sensor
  /// @return __double__ The current rotations
  virtual double getRotations() = 0;

  /// @brief Sets the rotations on the rotation sensor
  /// @param rotations __double__ The new amount of rotations
  virtual void setRotations(double rotations) = 0;

  /// @brief Gets the angle of the rotation sensor
  /// @return __double__ The angle of the rotation sensor, in radians
  virtual double getAngle() = 0;
};
}  // namespace io
}  // namespace driftless
#endif