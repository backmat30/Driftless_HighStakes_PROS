#ifndef __ARM_SUBSYSTEM_HPP__
#define __ARM_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/arm/IArmMotion.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class ArmSubsystem : public ASubsystem {
 private:
  // the arm motion controller
  std::unique_ptr<IArmMotion> m_arm_motion{};

 public:
  // constructs a new arm subsystem
  ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(ESubsystemCommand command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif