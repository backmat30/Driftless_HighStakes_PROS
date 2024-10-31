#include "pvegas/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
PistonHeightControlBuilder* PistonHeightControlBuilder::withPistons(
    pvegas::hal::PistonGroup pistons) {
  m_pistons = pistons;
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