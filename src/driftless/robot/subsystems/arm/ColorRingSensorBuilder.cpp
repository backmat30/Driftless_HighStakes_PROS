#include "driftless/robot/subsystems/arm/ColorRingSensorBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
ColorRingSensorBuilder* ColorRingSensorBuilder::withColorSensor(
    std::unique_ptr<driftless::io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
  return this;
}

ColorRingSensorBuilder* ColorRingSensorBuilder::withRingProximity(
    uint32_t ring_proximity) {
  m_ring_proximity = ring_proximity;
  return this;
}

std::unique_ptr<ColorRingSensor> ColorRingSensorBuilder::build() {
  std::unique_ptr<ColorRingSensor> color_ring_sensor{std::make_unique<ColorRingSensor>()};

  color_ring_sensor->setColorSensor(m_color_sensor);
  color_ring_sensor->setRingProximity(m_ring_proximity);

  return color_ring_sensor;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless