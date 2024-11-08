#include "driftless/robot/subsystems/odometry/DistancePositionResetterBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
DistancePositionResetterBuilder*
DistancePositionResetterBuilder::withDistanceSensor(
    std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor) {
  m_distance_sensor = std::move(distance_sensor);
  return this;
}

DistancePositionResetterBuilder* DistancePositionResetterBuilder::withLocalX(
    double local_x) {
  m_local_x = local_x;
  return this;
}

DistancePositionResetterBuilder* DistancePositionResetterBuilder::withLocalY(
    double local_y) {
  m_local_y = local_y;
  return this;
}

DistancePositionResetterBuilder*
DistancePositionResetterBuilder::withLocalTheta(double local_theta) {
  m_local_theta = local_theta;
  return this;
}

std::unique_ptr<IPositionResetter> DistancePositionResetterBuilder::build() {
  std::unique_ptr<DistancePositionResetter> position_resetter{
      std::make_unique<DistancePositionResetter>()};

  position_resetter->setDistanceSensor(m_distance_sensor);
  position_resetter->setLocalX(m_local_x);
  position_resetter->setLocalY(m_local_y);
  position_resetter->setLocalTheta(m_local_theta);

  return position_resetter;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless