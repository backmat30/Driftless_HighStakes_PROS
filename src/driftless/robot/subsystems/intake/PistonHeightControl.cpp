#include "driftless/robot/subsystems/intake/PistonHeightControl.hpp"
#include "pros/screen.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
void PistonHeightControl::init() {}

void PistonHeightControl::run() {}

void PistonHeightControl::setHeight(bool up) {
  if (!up) {
    m_height_pistons.extend();
  } else if (!m_secondary_pistons.isExtended()) {
    m_height_pistons.retract();
  }
  raised = !m_height_pistons.isExtended();

}

void PistonHeightControl::pullIn() {
  if (raised) {
    m_height_pistons.extend();
  }
  m_secondary_pistons.extend();
}

void PistonHeightControl::toggleSecondaryPistons() {
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 6, "TOGGLE CALLED");
  if (m_secondary_pistons.isExtended()) {
    pushOut();
  } else {
    pullIn();
  }
}

void PistonHeightControl::pushOut() { m_secondary_pistons.retract(); }

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