#ifndef __CLAMP_SUBSYSTEM_HPP__
#define __CLAMP_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/clamp/IClamp.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
class ClampSubsystem : public ASubsystem {
 private:
  // the clamp used by the subsystem
  std::unique_ptr<IClamp> m_clamp{};

 public:
  // constructs a new clamp subsystem object
  ClampSubsystem(std::unique_ptr<IClamp>& clamp);

  // initializes the subsystem
  void init() override;

  // runs the subsystem
  void run() override;

  // execute a given command
  void command(ESubsystemCommand command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif