#ifndef __E_SUBSYSTEM_HPP__
#define __E_SUBSYSTEM_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
enum class ESubsystem {
  ARM,
  CLAMP,
  DRIVETRAIN,
  ELEVATOR,
  INTAKE,
  ODOMETRY,
  RING_SORT
};
}
}  // namespace robot
}  // namespace driftless
#endif