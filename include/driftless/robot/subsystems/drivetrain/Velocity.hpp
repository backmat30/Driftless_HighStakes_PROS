#ifndef __VELOCITY_HPP__
#define __VELOCITY_HPP__
namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
struct Velocity {
  double left_velocity{};
  double right_velocity{};
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif