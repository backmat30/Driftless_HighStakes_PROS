#include "driftless/robot/subsystems/elevator/PistonRingRejectionBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
PistonRingRejectionBuilder* PistonRingRejectionBuilder::withPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<IRingRejection> PistonRingRejectionBuilder::build() {
  std::unique_ptr<PistonRingRejection> piston_ring_rejection{
      std::make_unique<PistonRingRejection>()};

  piston_ring_rejection->setPistons(m_pistons);

  return piston_ring_rejection;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless