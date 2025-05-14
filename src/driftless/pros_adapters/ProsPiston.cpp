#include "driftless/pros_adapters/ProsPiston.hpp"

namespace driftless {
namespace pros_adapters {
ProsPiston::ProsPiston(std::unique_ptr<pros::adi::DigitalOut>& adi_digital_out, bool extended_value)
    : m_adi_digital_out{std::move(adi_digital_out)}, m_extended_value{extended_value} {}

void ProsPiston::extend() {
  m_adi_digital_out->set_value(m_extended_value);
  extended = true;
}

void ProsPiston::retract() {
  m_adi_digital_out->set_value(!m_extended_value);
  extended = false;
}

void ProsPiston::toggleState() {
  extended = !extended;
  m_adi_digital_out->set_value(m_extended_value ^ extended);
}

bool ProsPiston::isExtended() {
  return extended;
}
}  // namespace pros_adapters
}  // namespace driftless