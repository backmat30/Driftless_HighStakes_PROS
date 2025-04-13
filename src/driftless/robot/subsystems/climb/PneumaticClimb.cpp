#include "driftless/robot/subsystems/climb/PneumaticClimb.hpp"

namespace driftless::robot::subsystems::climb {
void PneumaticClimb::init() {}

void PneumaticClimb::run() {}

void PneumaticClimb::toggleClimbing() {
  is_climbing = !is_climbing;
  if (is_climbing)
    m_stilt_pistons.extend();
  else
    m_stilt_pistons.retract();
}

void PneumaticClimb::pullBackClimber() {
  if(is_climbing)
  m_climber_pistons.retract();
}

void PneumaticClimb::pushForwardClimber() {
  if(is_climbing)
  m_climber_pistons.extend();
}

void PneumaticClimb::setStiltPistons(hal::PistonGroup& pistons) {
  m_stilt_pistons = pistons;
}

void PneumaticClimb::setClimberPistons(hal::PistonGroup& pistons) {
  m_climber_pistons = pistons;
}
}  // namespace driftless::robot::subsystems::climb
