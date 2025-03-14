#include "driftless/robot/subsystems/clamp/PistonClampBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
PistonClampBuilder* PistonClampBuilder::withPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<PistonClamp> PistonClampBuilder::build() {
  std::unique_ptr<PistonClamp> piston_clamp{std::make_unique<PistonClamp>()};
  piston_clamp->setPistons(m_pistons);

  return piston_clamp;
}
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless