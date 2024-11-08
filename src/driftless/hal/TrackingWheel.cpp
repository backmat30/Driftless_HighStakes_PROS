#include "driftless/hal/TrackingWheel.hpp"

namespace driftless {
namespace hal {
TrackingWheel::TrackingWheel(
    std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor,
    double wheel_radius)
    : m_rotation_sensor{std::move(rotation_sensor)},
      m_wheel_radius{wheel_radius} {}

void TrackingWheel::init() {
  if (m_rotation_sensor) {
    m_rotation_sensor->init();
  }
}

void TrackingWheel::reset() {
  if (m_rotation_sensor) {
    m_rotation_sensor->reset();
  }
}

double TrackingWheel::getDistance() {
  double distance{};
  if(m_rotation_sensor) {
    distance = m_rotation_sensor->getRotations() * m_wheel_radius;
  }

  return distance;
}

void TrackingWheel::setDistance(double distance) {
  if(m_rotation_sensor) {
    m_rotation_sensor->setRotations(distance / m_wheel_radius);
  }
}
}  // namespace hal
}  // namespace driftless