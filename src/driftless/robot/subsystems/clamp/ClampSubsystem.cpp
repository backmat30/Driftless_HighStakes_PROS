#include "driftless/robot/subsystems/clamp/ClampSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
ClampSubsystem::ClampSubsystem(std::unique_ptr<IClamp>& clamp)
    : m_clamp{std::move(clamp)}, ASubsystem{ESubsystem::CLAMP} {}

void ClampSubsystem::init() {}

void ClampSubsystem::run() {}

void ClampSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  if (command_name == ESubsystemCommand::CLAMP_SET_STATE) {
    bool state{static_cast<bool>(va_arg(args, int))};
    m_clamp->setState(state);
  }
}

void* ClampSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  switch (state_name ) {
    case ESubsystemState::CLAMP_GET_STATE:
    result = new bool{m_clamp->getState()};
    break;
    case ESubsystemState::CLAMP_HAS_GOAL:
    result = new bool{m_clamp->hasGoal()};
    break;
  }

  return result;
}
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless