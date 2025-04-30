#include "driftless/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
PistonHeightControlBuilder* PistonHeightControlBuilder::withHeightPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_height_pistons.addPiston(piston);
  return this;
}

PistonHeightControlBuilder* PistonHeightControlBuilder::withSecondaryPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_secondary_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<PistonHeightControl> PistonHeightControlBuilder::build() {
  std::unique_ptr<PistonHeightControl> height_control{std::make_unique<PistonHeightControl>()};
  height_control->setHeightPistons(m_height_pistons);
  height_control->setSecondaryPistons(m_secondary_pistons);

  return height_control;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless