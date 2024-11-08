#include "driftless/pros_adapters/ProsInertialSensor.hpp"

namespace driftless {
namespace pros_adapters {
ProsInertialSensor::ProsInertialSensor(
    std::unique_ptr<pros::IMU>& inertial_sensor, double tuning_constant)
    : m_inertial_sensor{std::move(inertial_sensor)},
      m_tuning_constant{tuning_constant} {}

void ProsInertialSensor::init() {
  if (m_inertial_sensor) {
    m_inertial_sensor->reset();
    // delay for the calibration time
    pros::delay(3000);
    m_inertial_sensor->set_data_rate(5);
  }
}

void ProsInertialSensor::reset() {
  if (m_inertial_sensor) {
    m_inertial_sensor->reset();
    // delay for the calibration time
    pros::delay(3000);
  }
}

double ProsInertialSensor::getHeading() {
  double heading{};
  if (m_inertial_sensor) {
    heading = m_inertial_sensor->get_heading() * DEGREES_TO_RADIANS *
              m_tuning_constant;
  }
  return heading;
}

double ProsInertialSensor::getRotation() {
  double rotation{};
  if (m_inertial_sensor) {
    rotation = m_inertial_sensor->get_rotation() * DEGREES_TO_RADIANS *
               m_tuning_constant;
  }
  return rotation;
}

void ProsInertialSensor::setHeading(double heading) {
  if (m_inertial_sensor) {
    m_inertial_sensor->set_heading(heading / DEGREES_TO_RADIANS);
  }
}

void ProsInertialSensor::setRotation(double rotation) {
  if(m_inertial_sensor) {
    m_inertial_sensor->set_rotation(rotation / DEGREES_TO_RADIANS);
  }
}
}  // namespace pros_adapters
}  // namespace driftless