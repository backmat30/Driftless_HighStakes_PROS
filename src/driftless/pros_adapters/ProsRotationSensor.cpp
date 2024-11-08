#include "driftless/pros_adapters/ProsRotationSensor.hpp"

namespace driftless {
namespace pros_adapters {
ProsRotationSensor::ProsRotationSensor(
    std::unique_ptr<pros::Rotation>& rotation_sensor)
    : m_rotation_sensor{std::move(rotation_sensor)} {}

void ProsRotationSensor::init() {
  if (m_rotation_sensor) {
    m_rotation_sensor->reset();
    m_rotation_sensor->set_data_rate(5);
  }
}

void ProsRotationSensor::reset() {
  if (m_rotation_sensor) {
    m_rotation_sensor->reset();
  }
}

double ProsRotationSensor::getRotations() {
  double rotations{};
  if (m_rotation_sensor) {
    rotations = m_rotation_sensor->get_position() * CENTIDEGREES_TO_RADIANS;
  }
  return rotations;
}

void ProsRotationSensor::setRotations(double rotations) {
  if (m_rotation_sensor) {
    m_rotation_sensor->set_position(rotations / CENTIDEGREES_TO_RADIANS);
  }
}

double ProsRotationSensor::getAngle() {
  double angle{};
  if (m_rotation_sensor) {
    angle = m_rotation_sensor->get_angle() * CENTIDEGREES_TO_RADIANS;
  }
  return angle;
}
}  // namespace pros_adapters
}  // namespace driftless