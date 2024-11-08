#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
class DirectIntakeBuilder {
 private:
  // the motors used for the intake
  driftless::hal::MotorGroup m_motors{};

 public:
  // adds motors to the builder
  DirectIntakeBuilder* withMotor(std::unique_ptr<driftless::io::IMotor>& motor);

  // build a new intake
  std::unique_ptr<DirectIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif