#ifndef __PROS_ROTATION_SENSOR_HPP__
#define __PROS_ROTATION_SENSOR_HPP__

#include <cmath>
#include <memory>

#include "pros/rotation.hpp"
#include "pvegas/io/IRotationSensor.hpp"

namespace pvegas {
namespace pros_adapters {
class ProsRotationSensor : public pvegas::io::IRotationSensor {
 private:
  // conversion factor between centidegrees and radians
  static constexpr double CENTIDEGREES_TO_RADIANS{M_PI / 18000};

  // the rotation sensor being adapted
  std::unique_ptr<pros::Rotation> m_rotation_sensor{};

 public:
  // constructor
  ProsRotationSensor(std::unique_ptr<pros::Rotation>& rotation_sensor);

  // initializes the rotational sensor
   void init() override;

  // resets the rotational sensor
   void reset() override;

  // retrieves the rotations done by the sensor
   double getRotations() override;

  // sets the rotations to a new value
   void setRotations(double rotations) override;

  // gets the angle the rotational sensor is at
   double getAngle() override;
};
}  // namespace pros_adapters
}  // namespace pvegas
#endif