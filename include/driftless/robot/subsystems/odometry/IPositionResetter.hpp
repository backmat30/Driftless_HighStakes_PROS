#ifndef __I_POSITION_RESETTER_HPP__
#define __I_POSITION_RESETTER_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
  // Interface to reset positions to absolute field position
class IPositionResetter {
 public:
  // destroyer
  virtual ~IPositionResetter() = default;

  // initialize the position resetter
  virtual void init() = 0;

  // run the position resetter
  virtual void run() = 0;

  // retrieves the x portion of the reset position
  virtual double getResetX(double theta) = 0;

  // retrieves the y portion of the reset position
  virtual double getResetY(double theta) = 0;

  // get the raw value of the resetter
  virtual double getRawValue() = 0;
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif