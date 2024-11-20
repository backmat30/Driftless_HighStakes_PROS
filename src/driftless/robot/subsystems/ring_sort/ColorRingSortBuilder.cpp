#include "driftless/robot/subsystems/ring_sort/ColorRingSortBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
ColorRingSortBuilder* ColorRingSortBuilder::withColorSensor(
    std::unique_ptr<driftless::io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
  return this;
}

ColorRingSortBuilder* ColorRingSortBuilder::withMaxRingDistance(
    double max_ring_distance) {
  m_max_ring_distance = max_ring_distance;
  return this;
}

ColorRingSortBuilder* ColorRingSortBuilder::withDistanceToElevatorEnd(
    double distance_to_elevator_end) {
  m_distance_to_elevator_end = distance_to_elevator_end;
  return this;
}

std::unique_ptr<IRingSort> ColorRingSortBuilder::build() {
  std::unique_ptr<ColorRingSort> color_ring_sort{
      std::make_unique<ColorRingSort>()};

  color_ring_sort->setColorSensor(m_color_sensor);
  color_ring_sort->setMaxRingDistance(m_max_ring_distance);
  color_ring_sort->setDistanceToElevatorEnd(m_distance_to_elevator_end);

  return color_ring_sort;
}
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless