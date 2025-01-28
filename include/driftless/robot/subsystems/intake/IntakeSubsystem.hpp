#ifndef __INTAKE_SUBSYSTEM_HPP__
#define __INTAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/intake/IHeightControl.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
class IntakeSubsystem : public ASubsystem {
 private:
  // the intake object
  std::unique_ptr<IIntake> m_intake{};

  // the intake height controller
  std::unique_ptr<IHeightControl> m_height_control{};

 public:
  // constructs a new intake subsystem
  IntakeSubsystem(std::unique_ptr<IIntake>& intake,
                  std::unique_ptr<IHeightControl>& height_control);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(ESubsystemCommand command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif