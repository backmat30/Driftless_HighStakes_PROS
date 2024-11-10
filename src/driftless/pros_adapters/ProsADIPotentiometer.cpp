#include "driftless/pros_adapters/ProsADIPotentiometer.hpp"

namespace driftless {
namespace pros_adapters {
ProsADIPotentiometer::ProsADIPotentiometer(
    std::unique_ptr<pros::adi::AnalogIn>& potentiometer, bool reversed)
    : m_potentiometer{std::move(potentiometer)}, m_reversed{reversed} {}

void ProsADIPotentiometer::init() { calibrate(); }

void ProsADIPotentiometer::calibrate() {
  double raw_position{m_potentiometer->get_value() * DECIDEGREES_TO_RADIANS};

  if (m_reversed) {
    position_offset = MAX_VALUE - raw_position;
  } else {
    position_offset = raw_position;
  }
}

double ProsADIPotentiometer::getAngle() {
  double position{};
  double raw_position{m_potentiometer->get_value() * DECIDEGREES_TO_RADIANS};

  if(m_reversed) {
    position = MAX_VALUE - raw_position;
  } else {
    position = raw_position;
  }

  return position - position_offset;
}
}  // namespace pros_adapters
}  // namespace driftless