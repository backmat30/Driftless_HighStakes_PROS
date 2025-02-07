#ifndef __I_INERTIAL_SENSOR_HPP__
#define __I_INERTIAL_SENSOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic inertial sensor
/// @author Matthew Backman
class IInertialSensor {
 public:
  /// @brief Destroys the inertial sensor
  virtual ~IInertialSensor() = default;

  /// @brief Initializes the inertial sensor
  virtual void init() = 0;

  /// @brief Resets the inertial sensor
  virtual void reset() = 0;

  /// @brief Gets the heading, in radians, bound between 0 and 2PI
  /// @return __double__ The heading
  virtual double getHeading() = 0;

  /// @brief Gets the rotation from the inertial sensor, in radians, unbounded
  /// @return __double__ The rotation
  virtual double getRotation() = 0;

  /// @brief Sets the heading of the inertial sensor
  /// @param heading __double__ The new heading, between 0 and 2PI
  virtual void setHeading(double heading) = 0;

  /// @brief Sets the rotation of the inertial sensor
  /// @param rotation __double__ The new rotation
  virtual void setRotation(double rotation) = 0;
};
}  // namespace io
}  // namespace driftless
#endif