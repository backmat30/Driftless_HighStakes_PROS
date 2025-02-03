#include "driftless/control/ControlSystem.hpp"

#include <cstdarg>
#include <memory>

namespace driftless {
namespace control {
void ControlSystem::addControl(std::unique_ptr<AControl>& control) {
  // moves the specified control from one spot in memory to one allocated to the
  // controls list
  controls.emplace(control->getName(), std::move(control));
}

bool ControlSystem::removeControl(EControl control) {
  // defines the return variable
  bool removed{false};
  // removes the control if it exists
  removed = controls.erase(control) > 0;
  return removed;
}

void ControlSystem::init() {
  // loop through list of controls and initializes them
  for (auto i{controls.begin()}; i != controls.end(); ++i) {
    i->second->init();
  }
}

void ControlSystem::run() {
  for (auto i{controls.begin()}; i != controls.end(); ++i) {
    i->second->run();
  }
}

void ControlSystem::pause() {
  // searches through the controls to find the active control and pauses it
  controls.at(active_control)->pause();
}

void ControlSystem::resume() {
  // searches through the controls to find the active control and resumes it
  controls.at(active_control)->resume();
}

void ControlSystem::sendCommand(EControl control_name,
                                EControlCommand command_name, ...) {
  // pauseis the current control if its not the desired control, then sets the
  // active control to the desired one
  if (control_name != active_control) {
    pause();
    active_control = control_name;
  }

  // creates and assigns a va_list for possible command arguments
  va_list args;
  va_start(args, command_name);
  // finds desired control and sends the specified command
  controls.at(control_name)->command(command_name, args);
  // ends the variable list
  va_end(args);
}

void* ControlSystem::getState(EControl control_name, EControlState state_name) {
  // defines the return variable
  void* state{nullptr};
  // find the desired control and returns the status of the given state
  state = controls.at(control_name)->state(state_name);
  return state;
}
}  // namespace control
}  // namespace driftless