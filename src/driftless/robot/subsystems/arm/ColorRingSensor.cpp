#include "driftless/robot/subsystems/arm/ColorRingSensor.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
void ColorRingSensor::init() { m_color_sensor->init(); }

void ColorRingSensor::run() {}

bool ColorRingSensor::hasRing() {
  uint32_t sensor_proximity{m_color_sensor->getProximity()};

  return sensor_proximity <= m_ring_proximity;
}

double ColorRingSensor::getHue() { return m_color_sensor->getHue(); }

void ColorRingSensor::setColorSensor(
    std::unique_ptr<driftless::io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
}

void ColorRingSensor::setRingProximity(uint32_t ring_proximity) {
  m_ring_proximity = ring_proximity;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless