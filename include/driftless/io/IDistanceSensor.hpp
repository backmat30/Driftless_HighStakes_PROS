#ifndef __I_DISTANCE_SENSOR_HPP__
#define __I_DISTANCE_SENSOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {
/// @brief Interface for a generic distance sensor
/// @author Matthew Backman
class IDistanceSensor {
 public:
  /// @brief Destroys the distance sensor
  virtual ~IDistanceSensor() = default;

  /// @brief Initializes the distance sensor
  virtual void init() = 0;

  /// @brief Resets the distance sensor
  virtual void reset() = 0;

  /// @brief Gets the distance, in inches, from the distance sensor
  /// @return __double__ The distance
  virtual double getDistance() = 0;
};
}  // namespace io
}  // namespace driftless
#endif