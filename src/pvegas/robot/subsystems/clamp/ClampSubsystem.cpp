#include "pvegas/robot/subsystems/clamp/ClampSubsystem.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace clamp {
ClampSubsystem::ClampSubsystem(std::unique_ptr<IClamp>& clamp)
    : m_clamp{std::move(clamp)} {}

void ClampSubsystem::init() {

}

void ClampSubsystem::run() {

}

void ClampSubsystem::command(std::string command_name, va_list& args) {
  if(command_name == SET_STATE_COMMAND_NAME) {
    bool state{static_cast<bool>(va_arg(args, int))};
    m_clamp->setState(state);
  }
}

void* ClampSubsystem::state(std::string state_name) {
  void* result{nullptr};
  if(state_name == GET_STATE_STATE_NAME) {
    result = new bool{m_clamp->getState()};
  }

  return result;
}
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas