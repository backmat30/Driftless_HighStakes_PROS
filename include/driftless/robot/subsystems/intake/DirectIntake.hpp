#ifndef __DIRECT_INTAKE_HPP__
#define __DIRECT_INTAKE_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
class DirectIntake : public IIntake {
 private:
  // group of motors used to run the intake
  driftless::hal::MotorGroup m_motors{};

 public:
  // initialize the intake
  void init() override;

  // run the intake
  void run() override;

  // sets the voltage of the intake motors
  void setVoltage(double voltage) override;

  // set the motors
  void setMotors(driftless::hal::MotorGroup& motors);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif