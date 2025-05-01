#include "driftless/robot/subsystems/clamp/PistonClamp.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
void PistonClamp::init() {}

void PistonClamp::run() {}

void PistonClamp::setState(bool clamped) {
  if (clamped) {
    m_pistons.extend();
  } else {
    m_pistons.retract();
  }
  state = clamped;
}

bool PistonClamp::getState() { return state; }

bool PistonClamp::hasGoal() {
  bool has_goal{};

  if (m_distance_sensor) {
    has_goal = m_distance_sensor->getDistance() <= m_distance_to_goal;
  }

  return has_goal;
}

void PistonClamp::setPistons(driftless::hal::PistonGroup& pistons) {
  m_pistons = std::move(pistons);
}

void PistonClamp::setDistanceSensor(
    std::unique_ptr<io::IDistanceSensor>& distance_sensor) {
  m_distance_sensor = std::move(distance_sensor);
}

void PistonClamp::setDistanceToGoal(double distance_to_goal) {
  m_distance_to_goal = distance_to_goal;
}
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless