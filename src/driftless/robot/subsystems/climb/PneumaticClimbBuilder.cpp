#include "driftless/robot/subsystems/climb/PneumaticClimbBuilder.hpp"

namespace driftless::robot::subsystems::climb {
PneumaticClimbBuilder* PneumaticClimbBuilder::withStiltPiston(
    std::unique_ptr<io::IPiston>& stilt_piston) {
  m_stilt_pistons.addPiston(stilt_piston);
  return this;
}

PneumaticClimbBuilder* PneumaticClimbBuilder::withClimberPiston(
    std::unique_ptr<io::IPiston>& climber_piston) {
  m_climber_pistons.addPiston(climber_piston);
  return this;
}

std::unique_ptr<IClimb> PneumaticClimbBuilder::build() {
  std::unique_ptr<PneumaticClimb> climb{std::make_unique<PneumaticClimb>()};
  climb->setStiltPistons(m_stilt_pistons);
  climb->setClimberPistons(m_climber_pistons);
  return climb;
}
}  // namespace driftless::robot::subsystems::climb