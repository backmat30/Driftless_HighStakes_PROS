#include "driftless/control/ControlSystem.hpp"

#include <cstdarg>
#include <memory>

namespace driftless {
namespace control {
void ControlSystem::addControl(std::unique_ptr<AControl>& control) {
  // moves the specified control from one spot in memory to one allocated to the
  // controls list
  controls.push_back(std::move(control));
}

bool ControlSystem::removeControl(std::string control) {
  // defines the return variable
  bool removed{false};
  // loops through the list of controls until the specified control is found
  for (auto i{controls.begin()}; i != controls.end(); ++i) {
    if ((*i)->getName() == control) {
      // removes the control from the list and clears the memory allocated
      controls.erase(i);
      removed = true;
      break;
    }
  }
  return removed;
}

void ControlSystem::init() {
  // loop through list of controls and initializes them
  for (auto& control : controls) {
    control->init();
  }
}

void ControlSystem::run() {
  for (auto& control : controls) {
    control->run();
  }
}

void ControlSystem::pause() {
  // searches through the controls to find the active control and pauses it
  for (auto& control : controls) {
    if (control->getName() == active_control) {
      control->pause();
      break;
    }
  }
}

void ControlSystem::resume() {
  // searches through the controls to find the active control and resumes it
  for (auto& control : controls) {
    if (control->getName() == active_control) {
      control->resume();
      break;
    }
  }
}

void ControlSystem::sendCommand(std::string control_name,
                                std::string command_name, ...) {
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
  for (auto& control : controls) {
    if (control->getName() == control_name) {
      control->command(command_name, args);
      // resumes the given task
      control->resume();
      break;
    }
  }
  // ends the variable list
  va_end(args);
}

void* ControlSystem::getState(std::string control_name,
                              std::string state_name) {
  // defines the return variable
  void* state{nullptr};
  // find the desired control and returns the status of the given state
  for (auto& control : controls) {
    if (control->getName() == control_name) {
      state = control->state(state_name);
      break;
    }
  }
  return state;
}
}  // namespace control
}  // namespace driftless