#ifndef __I_POTENTIOMETER_HPP__
#define __I_POTENTIOMETER_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic potentiometer
/// @author Matthew Backman
class IPotentiometer {
 public:
  /// @brief Deletes the potentiometer
  virtual ~IPotentiometer() = default;

  /// @brief Initializes the potentiometer
  virtual void init() = 0;

  /// @brief Calibrates the potentiometer
  virtual void calibrate() = 0;

  /// @brief Gets the angle of the potentiometer
  /// @return __double__ The angle, in radians
  virtual double getAngle() = 0;
};
}  // namespace io
}  // namespace driftless
#endif