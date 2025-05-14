#include "driftless/robot/subsystems/ring_sort/ColorRingSort.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
void ColorRingSort::init() { m_color_sensor->init(); }

void ColorRingSort::run() {}

double ColorRingSort::getRingHue() { return m_color_sensor->getHue(); }

io::RGBValue ColorRingSort::getRingRGB() {
  return m_color_sensor->getRGB();
}

double ColorRingSort::getDistanceToElevatorEnd() {
  return m_distance_to_elevator_end;
}

bool ColorRingSort::hasRing() {
  return m_color_sensor->getProximity() >= m_max_ring_distance;
}

void ColorRingSort::setColorSensor(
    std::unique_ptr<driftless::io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
}

void ColorRingSort::setMaxRingDistance(double max_ring_distance) {
  m_max_ring_distance = max_ring_distance;
}

void ColorRingSort::setDistanceToElevatorEnd(double distance_to_elevator_end) {
  m_distance_to_elevator_end = distance_to_elevator_end;
}
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless