#include "driftless/hal/PistonGroup.hpp"

namespace driftless {
namespace hal {
void PistonGroup::addPiston(std::unique_ptr<driftless::io::IPiston>& piston) {
  m_pistons.push_back(std::move(piston));
}

void PistonGroup::extend() {
  extended = true;
  for (std::unique_ptr<io::IPiston>& piston : m_pistons) {
    if (piston) {
      piston->extend();
    }
  }
}

void PistonGroup::retract() {
  extended = false;
  for (std::unique_ptr<io::IPiston>& piston : m_pistons) {
    if (piston) {
      piston->retract();
    }
  }
}

void PistonGroup::toggleState() {
  extended = !extended;
  if (extended) {
    extend();
  } else {
    retract();
  }
}

bool PistonGroup::isExtended() { return extended; }
}  // namespace hal
}  // namespace driftless