#ifndef __I_ELEVATOR_HPP__
#define __I_ELEVATOR_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
class IElevator {
 public:
  // destroyer
  virtual ~IElevator() = default;

  // initialize the elevator
  virtual void init() = 0;

  // run the elevator
  virtual void run() = 0;

  // set the voltage of the elevator motors
  virtual void setVoltage(double voltage) = 0;

  // set the position of the elevator
  virtual void setPosition(double position) = 0;

  // get the position of the elevator
  virtual double getPosition() = 0;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif