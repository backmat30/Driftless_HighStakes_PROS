#include "pvegas/robot/subsystems/intake/DirectIntake.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
void DirectIntake::init() {}

void DirectIntake::run() {}

void DirectIntake::setVoltage(double voltage) { m_motors.setVoltage(voltage); }

void DirectIntake::setMotors(driftless::hal::MotorGroup& motors) {
  m_motors = motors;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas