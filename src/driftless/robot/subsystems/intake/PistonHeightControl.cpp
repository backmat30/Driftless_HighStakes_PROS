#include "driftless/robot/subsystems/intake/PistonHeightControl.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
void PistonHeightControl::init() {}

void PistonHeightControl::run() {}

void PistonHeightControl::setHeight(bool up) {
  if (up) {
    m_height_pistons.extend();
  } else if (!m_secondary_pistons.isExtended()) {
    m_height_pistons.retract();
  }
  raised = m_height_pistons.isExtended();
}

void PistonHeightControl::pullIn() {
  if (m_height_pistons.isExtended()) {
    m_secondary_pistons.retract();
  }
}

void PistonHeightControl::toggleSecondaryPistons() {
  if (m_secondary_pistons.isExtended()) {
    m_secondary_pistons.retract();
  } else {
    m_secondary_pistons.extend();
  }
}

void PistonHeightControl::pushOut() { m_secondary_pistons.extend(); }

bool PistonHeightControl::isRaised() { return raised; }

void PistonHeightControl::setHeightPistons(
    driftless::hal::PistonGroup& pistons) {
  m_height_pistons = std::move(pistons);
}

void PistonHeightControl::setSecondaryPistons(
    driftless::hal::PistonGroup& pistons) {
  m_secondary_pistons = std::move(pistons);
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless