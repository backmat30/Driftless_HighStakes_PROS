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

PistonClampBuilder* PistonClampBuilder::withDistanceSensor(
    std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor) {
  m_distance_sensor = std::move(distance_sensor);
  return this;
}

PistonClampBuilder* PistonClampBuilder::withDistanceToGoal(
    double distance_to_goal) {
  m_distance_to_goal = distance_to_goal;
  return this;
}

std::unique_ptr<PistonClamp> PistonClampBuilder::build() {
  std::unique_ptr<PistonClamp> piston_clamp{std::make_unique<PistonClamp>()};
  piston_clamp->setPistons(m_pistons);
  piston_clamp->setDistanceSensor(m_distance_sensor);
  piston_clamp->setDistanceToGoal(m_distance_to_goal);

  return piston_clamp;
}
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless