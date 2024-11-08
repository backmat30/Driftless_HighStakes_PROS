#ifndef __I_DISTANCE_SENSOR_HPP__
#define __I_DISTANCE_SENSOR_HPP__

namespace driftless {
namespace io {
class IDistanceSensor {
 public:
  // destroyer
  virtual ~IDistanceSensor() = default;

  // initialize the distance sensor
  virtual void init() = 0;

  // resets the distance sensor
  virtual void reset() = 0;

  // gets the distance from the sensor
  virtual double getDistance() = 0;
};
}  // namespace io
}  // namespace driftless
#endif