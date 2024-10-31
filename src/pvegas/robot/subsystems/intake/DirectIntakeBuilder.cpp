#include "pvegas/robot/subsystems/intake/DirectIntakeBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
DirectIntakeBuilder* DirectIntakeBuilder::withMotors(
    pvegas::hal::MotorGroup motors) {
  m_motors = motors;
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