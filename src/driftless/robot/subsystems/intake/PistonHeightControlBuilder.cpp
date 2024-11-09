#include "driftless/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
PistonHeightControlBuilder* PistonHeightControlBuilder::withPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<PistonHeightControl> PistonHeightControlBuilder::build() {
  std::unique_ptr<PistonHeightControl> height_control{std::make_unique<PistonHeightControl>()};
  height_control->setPistons(m_pistons);

  return height_control;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless