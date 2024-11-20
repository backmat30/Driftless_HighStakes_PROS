#include "driftless/robot/subsystems/elevator/PistonRingRejection.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
void PistonRingRejection::init() {}

void PistonRingRejection::run() {}

void PistonRingRejection::deploy() { m_pistons.extend(); }

void PistonRingRejection::retract() { m_pistons.retract(); }

bool PistonRingRejection::isDeployed() { return m_pistons.isExtended(); }

void PistonRingRejection::setPistons(driftless::hal::PistonGroup& pistons) {
  m_pistons = std::move(pistons);
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless