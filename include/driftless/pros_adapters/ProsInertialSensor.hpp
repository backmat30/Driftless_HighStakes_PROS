#ifndef __PROS_INERTIAL_SENSOR_HPP__
#define __PROS_INERTIAL_SENSOR_HPP__

#include <math.h>

#include <memory>

#include "driftless/io/IInertialSensor.hpp"
#include "pros/imu.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief The class for adapting a PROS inertial sensor
/// @author Matthew Backman
class ProsInertialSensor : public driftless::io::IInertialSensor {
 private:
  // conversion factor from degrees to radians, also flips direction
  static constexpr double DEGREES_TO_RADIANS{-M_PI / 180};

  // inertial sensor being adapted
  std::unique_ptr<pros::IMU> m_inertial_sensor{};

  // tuning constant to ensure accuracy
  double m_tuning_constant{};

 public:
  /// @brief Constructs a new ProsInertialSensor object
  /// @param inertial_sensor The inertial sensor being adapted
  /// @param tuning_constant The tuning constant to ensure accuracy
  ProsInertialSensor(std::unique_ptr<pros::IMU>& inertial_sensor,
                     double tuning_constant = 1);

  /// @brief Initializes the inertial sensor
  void init() override;

  /// @brief Resets the inertial sensor
  void reset() override;

  /// @brief Gets the rotation around the Z axis (bound to [0, 360))
  /// @return __double__ The heading of the inertial sensor
  double getHeading() override;

  /// @brief Gets the unbounded rotation around the Z axis
  /// @return __double__ The rotation of the inertial sensor
  double getRotation() override;

  /// @brief Sets the heading to a new value
  /// @param heading The new heading value
  void setHeading(double heading) override;

  /// @brief Sets the rotation to a new value
  /// @param rotation The new rotation value
  void setRotation(double rotation) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif