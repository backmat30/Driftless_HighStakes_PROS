#include "pvegas/robot/subsystems/intake/DirectIntakeBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
DirectIntakeBuilder* DirectIntakeBuilder::withMotor(
    std::unique_ptr<driftless::io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

std::unique_ptr<DirectIntake> DirectIntakeBuilder::build() {
  std::unique_ptr<DirectIntake> intake{};
  intake->setMotors(m_motors);

  return intake;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas