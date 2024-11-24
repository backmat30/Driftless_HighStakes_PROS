#ifndef __PROS_INERTIAL_SENSOR_HPP__
#define __PROS_INERTIAL_SENSOR_HPP__

#include <math.h>

#include <memory>

#include "pros/imu.hpp"
#include "driftless/io/IInertialSensor.hpp"

namespace driftless {
namespace pros_adapters {
class ProsInertialSensor : public driftless::io::IInertialSensor {
 private:
  // conversion factor from degrees to radians, also flips direction
  static constexpr double DEGREES_TO_RADIANS{-M_PI / 180};

  // inertial sensor being adapted
  std::unique_ptr<pros::IMU> m_inertial_sensor{};

  // tuning constant to ensure accuracy
  double m_tuning_constant{};

 public:
  // constructor
  ProsInertialSensor(std::unique_ptr<pros::IMU>& inertial_sensor,
                     double tuning_constant = 1);

  // initialize the inertial sensor
  void init() override;

  // reset the inertial sensor
  void reset() override;

  // gets the rotation around the Z axis (bound to [0, 360))
  double getHeading() override;

  // get the unbounded rotation around the Z axis
  double getRotation() override;

  // sets the heading to a new value
  void setHeading(double heading) override;

  // sets the rotation to a new value
  void setRotation(double rotation) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif