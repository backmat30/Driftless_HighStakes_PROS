#include "pvegas/robot/subsystems/clamp/PistonClampBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace clamp {
PistonClampBuilder* PistonClampBuilder::withPiston(std::unique_ptr<pvegas::io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<PistonClamp> PistonClampBuilder::build() {
  std::unique_ptr<PistonClamp> piston_clamp{};
  piston_clamp->setPistons(m_pistons);

  return piston_clamp;
}
}
}
}
}