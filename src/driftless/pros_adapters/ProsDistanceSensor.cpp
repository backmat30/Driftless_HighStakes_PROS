#include "driftless/pros_adapters/ProsDistanceSensor.hpp"

namespace driftless {
namespace pros_adapters {
ProsDistanceSensor::ProsDistanceSensor(
    std::unique_ptr<pros::Distance>& distance_sensor,
    double tuning_constant, double tuning_offset)
    : m_distance_sensor{std::move(distance_sensor)},
      m_tuning_constant{tuning_constant},
      m_tuning_offset{tuning_offset} {}

void ProsDistanceSensor::init() {}

void ProsDistanceSensor::reset() {}

double ProsDistanceSensor::getDistance() {
  double distance{};
  if (m_distance_sensor) {
    distance = (m_distance_sensor->get() * MM_TO_INCHES * m_tuning_constant) +
               m_tuning_offset;
  }
  return distance;
}
}  // namespace pros_adapters
}  // namespace driftless