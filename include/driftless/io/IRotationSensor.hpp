#ifndef __I_ROTATION_SENSOR_HPP__
#define __I_ROTATION_SENSOR_HPP__

namespace driftless {
namespace io {
class IRotationSensor {
 public:
  // destroyer
  virtual ~IRotationSensor() = default;

  // initializes the rotational sensor
  virtual void init() = 0;

  // resets the rotational sensor
  virtual void reset() = 0;

  // retrieves the rotations done by the sensor
  virtual double getRotations() = 0;

  // sets the rotations to a new value
  virtual void setRotations(double rotations) = 0;

  // gets the angle the rotational sensor is at
  virtual double getAngle() = 0;
};
}  // namespace io
}  // namespace driftless
#endif