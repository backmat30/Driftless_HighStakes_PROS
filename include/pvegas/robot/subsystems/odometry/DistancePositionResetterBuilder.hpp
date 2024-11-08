#ifndef __DISTANCE_POSITION_RESETTER_BUILDER_HPP__
#define __DISTANCE_POSITION_RESETTER_BUILDER_HPP__

#include <memory>

#include "pvegas/robot/subsystems/odometry/DistancePositionResetter.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
class DistancePositionResetterBuilder {
 private:
  // the resetter being built
  std::unique_ptr<driftless::io::IDistanceSensor> m_distance_sensor{};

  // the local X offset
  double m_local_x{};

  // the local y offset
  double m_local_y{};

  // the local angular offset
  double m_local_theta{};

 public:
  // add a distance sensor to the position resetter
  DistancePositionResetterBuilder* withDistanceSensor(std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor);

  // add a local x offset to the position resetter
  DistancePositionResetterBuilder* withLocalX(double local_x);

  // add a local y offset to the position resetter
  DistancePositionResetterBuilder* withLocalY(double local_y);

  // add a local angular offset to the position resetter
  DistancePositionResetterBuilder* withLocalTheta(double local_theta);

  // build the position resetter
  std::unique_ptr<IPositionResetter> build();
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif