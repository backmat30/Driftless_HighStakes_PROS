#include "driftless/robot/subsystems/climb/PneumaticClimbBuilder.hpp"

namespace driftless::robot::subsystems::climb {
PneumaticClimbBuilder* PneumaticClimbBuilder::withStiltPistons(
    hal::PistonGroup& stilt_pistons) {
  m_stilt_pistons = stilt_pistons;
  return this;
}

PneumaticClimbBuilder* PneumaticClimbBuilder::withClimberPistons(
    hal::PistonGroup& climber_pistons) {
  m_climber_pistons = climber_pistons;
  return this;
}

std::unique_ptr<IClimb> PneumaticClimbBuilder::build() {
  std::unique_ptr<PneumaticClimb> climb{std::make_unique<PneumaticClimb>()};
  climb->setStiltPistons(m_stilt_pistons);
  climb->setClimberPistons(m_climber_pistons);
  return climb;
}
}  // namespace driftless::robot::subsystems::climb