#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include "pvegas/robot/subsystems/intake/DirectIntake.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
class DirectIntakeBuilder {
 private:
  // the motors used for the intake
  pvegas::hal::MotorGroup m_motors{};

 public:
  // adds motors to the builder
  DirectIntakeBuilder* withMotor(std::unique_ptr<pvegas::io::IMotor>& motor);

  // build a new intake
  std::unique_ptr<DirectIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif