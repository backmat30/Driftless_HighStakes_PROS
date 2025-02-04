#ifndef __PROS_ROTATION_SENSOR_HPP__
#define __PROS_ROTATION_SENSOR_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IRotationSensor.hpp"
#include "pros/rotation.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief The class for adapting a PROS rotation sensor
/// @author Matthew Backman
class ProsRotationSensor : public driftless::io::IRotationSensor {
 private:
  // conversion factor between centidegrees and radians
  static constexpr double CENTIDEGREES_TO_RADIANS{M_PI / 18000};

  // the rotation sensor being adapted
  std::unique_ptr<pros::Rotation> m_rotation_sensor{};

 public:
  /// @brief Constructs a new ProsRotationSensor object
  /// @param rotation_sensor __std::unique_ptr<pros::Rotation>&__ The rotation
  /// sensor being adapted
  ProsRotationSensor(std::unique_ptr<pros::Rotation>& rotation_sensor);

  /// @brief Initializes the rotational sensor
  void init() override;

  /// @brief Resets the rotational sensor
  void reset() override;

  /// @brief Retrieves the rotations done by the sensor
  /// @return __double__ The rotations done by the sensor
  double getRotations() override;

  /// @brief Sets the rotations to a new value
  /// @param rotations The new rotations value
  void setRotations(double rotations) override;

  /// @brief Gets the angle the rotational sensor is at
  /// @return __double__ The angle of the rotational sensor
  double getAngle() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif