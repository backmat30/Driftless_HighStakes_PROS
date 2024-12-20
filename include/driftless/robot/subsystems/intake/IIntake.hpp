#ifndef __I_INTAKE_HPP__
#define __I_INTAKE_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
class IIntake {
 public:
  // destroys the intake object
  virtual ~IIntake() = default;

  // initialize the intake
  virtual void init() = 0;

  // run the intake
  virtual void run() = 0;

  // set the voltage of the motor
  virtual void setVoltage(double voltage) = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif