#ifndef __CLAMP_SUBSYSTEM_HPP__
#define __CLAMP_SUBSYSTEM_HPP__

#include <memory>

#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/clamp/IClamp.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
class ClampSubsystem : public ASubsystem {
 private:
  // the name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"CLAMP"};

  // the name of the set state command
  static constexpr char SET_STATE_COMMAND_NAME[]{"SET STATE"};

  // the name of the clamp state
  static constexpr char GET_STATE_STATE_NAME[]{"GET STATE"};

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
  void command(std::string command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif