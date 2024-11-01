#include "pvegas/robot/subsystems/intake/PistonHeightControl.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
void PistonHeightControl::init() {}

void PistonHeightControl::run() {}

void PistonHeightControl::setHeight(bool up) {
  if (up) {
    m_pistons.retract();
  } else {
    m_pistons.extend();
  }
  raised = up;
}

bool PistonHeightControl::isRaised() { return raised; }

void PistonHeightControl::setPistons(pvegas::hal::PistonGroup pistons) {
  m_pistons = pistons;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas