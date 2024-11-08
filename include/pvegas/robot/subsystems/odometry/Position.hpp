#ifndef __POSITION_HPP__
#define __POSITION_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
struct Position {
  // x coordinate
  double x{};

  // y coordinate
  double y{};

  // angle
  double theta{};

  // x velocity
  double xV{};

  // y velocity
  double yV{};

  // angular velocity
  double thetaV{};
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif