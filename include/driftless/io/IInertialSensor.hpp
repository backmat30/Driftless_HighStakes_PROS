#ifndef __I_INERTIAL_SENSOR_HPP__
#define __I_INERTIAL_SENSOR_HPP__

namespace driftless {
namespace io {
class IInertialSensor {
 public:
  // destroyer
  virtual ~IInertialSensor() = default;

  // initialize the inertial sensor
  virtual void init() = 0;

  // reset the inertial sensor
  virtual void reset() = 0;

  // gets the rotation around Z axis (bound to [0, 360))
  virtual double getHeading() = 0;

  // gets the rotation around the Z axis (unbounded)
  virtual double getRotation() = 0;

  // sets the heading to a new value
  virtual void setHeading(double heading) = 0;

  // sets the rotation to a new value
  virtual void setRotation(double rotation) = 0;
};
}  // namespace io
}  // namespace driftless
#endif