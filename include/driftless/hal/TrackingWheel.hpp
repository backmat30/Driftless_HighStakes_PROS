#ifndef __TRACKING_WHEEL_HPP__
#define __TRACKING_WHEEL_HPP__

#include <memory>

#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IRotationSensor.hpp"

namespace driftless {
namespace hal {
class TrackingWheel : public driftless::io::IDistanceTracker {
 private:
  // the rotation sensor for the wheel
  std::unique_ptr<driftless::io::IRotationSensor> m_rotation_sensor{};

  // the radius of the wheel
  double m_wheel_radius{};

 public:
  // constructor
  TrackingWheel(std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor,
                double wheel_radius);

  // initialize the sensor
  void init() override;

  // reset the sensor
  void reset() override;

  // gets the distance travelled by the wheel
  double getDistance() override;

  // sets the distance to a new value
  void setDistance(double distance) override;
};
}  // namespace hal
}  // namespace driftless
#endif