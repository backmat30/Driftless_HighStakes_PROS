#include "pvegas/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
PistonHeightControlBuilder* PistonHeightControlBuilder::withPiston(
    std::unique_ptr<pvegas::io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<PistonHeightControl> PistonHeightControlBuilder::build() {
  std::unique_ptr<PistonHeightControl> height_control{};
  height_control->setPistons(m_pistons);

  return height_control;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas