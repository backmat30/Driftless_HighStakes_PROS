#ifndef __COLOR_RING_SENSOR_HPP__
#define __COLOR_RING_SENSOR_HPP__

#include <memory>

#include "driftless/io/IColorSensor.hpp"
#include "driftless/robot/subsystems/arm/IRingSensor.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class ColorRingSensor : public IRingSensor {
 private:
  // the color sensor used
  std::unique_ptr<driftless::io::IColorSensor> m_color_sensor{};

  // the proximity value from the sensor to be considered "holding a ring"
  uint32_t m_ring_proximity{};

 public:
  // initialize the ring sensor
  void init() override;

  // run the ring sensor
  void run() override;

  // determines if there is a ring in front of the sensor
  bool hasRing() override;

  // determines the hue from the color sensor
  double getHue() override;

  // sets the color sensor
  void setColorSensor(std::unique_ptr<driftless::io::IColorSensor>& color_sensor);

  // sets the ring proximity value
  void setRingProximity(uint32_t ring_proximity);
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif