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

void PneumaticClimb::pushOutPassiveHooks() {
  if(is_climbing)
  m_passive_hook_pistons.extend();
}

void PneumaticClimb::pullInPassiveHooks() {
  if(is_climbing)
  m_passive_hook_pistons.retract();
}

bool PneumaticClimb::arePassivesOut() {
  return m_passive_hook_pistons.isExtended();
}

bool PneumaticClimb::isClimbing() {
  return is_climbing;
}

void PneumaticClimb::setStiltPistons(hal::PistonGroup& pistons) {
  m_stilt_pistons = std::move(pistons);
}

void PneumaticClimb::setClimberPistons(hal::PistonGroup& pistons) {
  m_climber_pistons = std::move(pistons);
}

void PneumaticClimb::setPassiveHookPistons(hal::PistonGroup& pistions) {
  m_passive_hook_pistons = std::move(pistions);
}
}  // namespace driftless::robot::subsystems::climb
