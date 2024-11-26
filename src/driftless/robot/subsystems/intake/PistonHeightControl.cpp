#include "driftless/robot/subsystems/intake/PistonHeightControl.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
void PistonHeightControl::init() {}

void PistonHeightControl::run() {}

void PistonHeightControl::setHeight(bool up) {
  if (up) {
    m_pistons.extend();
  } else {
    m_pistons.retract();
  }
  raised = up;
}

bool PistonHeightControl::isRaised() { return raised; }

void PistonHeightControl::setPistons(driftless::hal::PistonGroup& pistons) {
  m_pistons = std::move(pistons);
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless