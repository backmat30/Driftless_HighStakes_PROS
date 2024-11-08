#include "driftless/pros_adapters/ProsADIPotentiometer.hpp"

namespace driftless {
namespace pros_adapters {
ProsADIPotentiometer::ProsADIPotentiometer(
    std::unique_ptr<pros::adi::AnalogIn>& potentiometer)
    : m_potentiometer{std::move(potentiometer)} {}

void ProsADIPotentiometer::init() { calibrate(); }

void ProsADIPotentiometer::calibrate() {
  m_potentiometer->calibrate();
  pros::delay(500);
}

double ProsADIPotentiometer::getAngle() {
  int32_t decidegrees{m_potentiometer->get_value()};
  return decidegrees * DECIDEGREES_TO_RADIANS;
}
}  // namespace pros_adapters
}  // namespace driftless