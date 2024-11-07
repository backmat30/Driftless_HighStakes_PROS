#include "pvegas/pros_adapters/ProsPiston.hpp"

namespace pvegas {
namespace pros_adapters {
ProsPiston::ProsPiston(std::unique_ptr<pros::adi::DigitalOut>& adi_digital_out)
    : m_adi_digital_out{std::move(adi_digital_out)} {}

void ProsPiston::extend() {
  m_adi_digital_out->set_value(true);
  extended = true;
}

void ProsPiston::retract() {
  m_adi_digital_out->set_value(false);
  extended = false;
}

void ProsPiston::toggleState() {
  extended = !extended;
  m_adi_digital_out->set_value(extended);
}

bool ProsPiston::isExtended() {
  return extended;
}
}  // namespace pros_adapters
}  // namespace pvegas