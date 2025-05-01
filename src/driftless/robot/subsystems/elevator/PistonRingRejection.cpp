#include "driftless/robot/subsystems/elevator/PistonRingRejection.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
void PistonRingRejection::init() {}

void PistonRingRejection::run() {}

void PistonRingRejection::deploy() {
  if (rejection_direction == ERejectionDirection::LEFT) {
    m_right_pistons.extend();
  } else if (rejection_direction == ERejectionDirection::RIGHT) {
    m_left_pistons.extend();
  } else {
    m_left_pistons.extend();
    m_right_pistons.extend();
  }
}

void PistonRingRejection::retract() {
  m_left_pistons.retract();
  m_right_pistons.retract();
}

void PistonRingRejection::setDeploymentDirection(
    ERejectionDirection direction) {
  rejection_direction = direction;

  switch(rejection_direction) {
    case ERejectionDirection::LEFT:
      m_left_pistons.retract();
      break;
    case ERejectionDirection::RIGHT:
      m_right_pistons.retract();
      break;
  }
}

bool PistonRingRejection::isDeployed() {
  return m_left_pistons.isExtended() || m_right_pistons.isExtended();
}

void PistonRingRejection::setLeftPistons(driftless::hal::PistonGroup& pistons) {
  m_left_pistons = std::move(pistons);
}

void PistonRingRejection::setRightPistons(
    driftless::hal::PistonGroup& pistons) {
  m_right_pistons = std::move(pistons);
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless