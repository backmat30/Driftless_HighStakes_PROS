#ifndef __COLOR_RING_SENSOR_BUILDER_HPP__
#define __COLOR_RING_SENSOR_BUILDER_HPP__

#include "pvegas/robot/subsystems/arm/ColorRingSensor.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class ColorRingSensorBuilder {
 private:
  // the color sensor used
  std::unique_ptr<driftless::io::IColorSensor> m_color_sensor{};

  // the proximity value from the sensor to be considered "holding a ring"
  uint32_t m_ring_proximity{};

 public:
  // add a color sensor to the builder
  ColorRingSensorBuilder* withColorSensor(
      std::unique_ptr<driftless::io::IColorSensor>& color_sensor);

  // add a proximity value to the builder
  ColorRingSensorBuilder* withRingProximity(uint32_t ring_proximity);

  // build a new ring sensor
  std::unique_ptr<ColorRingSensor> build();
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif