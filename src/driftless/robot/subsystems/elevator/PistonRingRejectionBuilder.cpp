#include "driftless/robot/subsystems/elevator/PistonRingRejectionBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
PistonRingRejectionBuilder* PistonRingRejectionBuilder::withLeftPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_left_pistons.addPiston(piston);
  return this;
}

PistonRingRejectionBuilder* PistonRingRejectionBuilder::withRightPiston(
    std::unique_ptr<driftless::io::IPiston>& piston) {
  m_right_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<IRingRejection> PistonRingRejectionBuilder::build() {
  std::unique_ptr<PistonRingRejection> piston_ring_rejection{
      std::make_unique<PistonRingRejection>()};

  piston_ring_rejection->setLeftPistons(m_left_pistons);
  piston_ring_rejection->setRightPistons(m_right_pistons);

  return piston_ring_rejection;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless