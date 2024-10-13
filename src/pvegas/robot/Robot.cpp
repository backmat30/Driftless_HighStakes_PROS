#include "pvegas/robot/Robot.hpp"

namespace pvegas {
namespace robot {
void Robot::addSubsystem(std::unique_ptr<ASubsystem>& subsystem) {
  subsystems.push_back(std::move(subsystem));
}

bool Robot::removeSubsystem(std::string subsystem) {
  // iterate through addresses rather than subsystems to know location to erase
  for (auto i{subsystems.begin()}; i != subsystems.end(); ++i) {
    // ptr to a ptr, only want single ptr
    if ((*i)->getName() == subsystem) {
      subsystems.erase(i);
      return true;
    }
  }
  return false;
}

void Robot::init() {
  // init all subsystems
  for (auto& subsystem : subsystems) {
    subsystem->init();
  }
}

void Robot::run() {
  // run all subsystems
  for (auto& subsystem : subsystems) {
    subsystem->run();
  }
}

void Robot::sendCommand(std::string subsystem_name, std::string command_name,
                        ...) {
  // variable list to store any extra parameters to be passed to the command
  va_list args;
  // tells the list to store anything past command_name
  va_start(args, command_name);
  // find correct subsystem
  for (auto& subsystem : subsystems) {
    if (subsystem->getName() == subsystem_name) {
      // tells subsystem to run specified command using params held in the
      // variable list
      subsystem->command(command_name, args);
      break;
    }
  }
  // closes the variable list
  va_end(args);
}

void* Robot::getState(std::string subsystem_name, std::string state_name) {
  // basic ptr init
  void* state{nullptr};
  // find correct subsystem
  for (auto& subsystem : subsystems) {
    if (subsystem->getName() == subsystem_name) {
      // gather state info then exit loop
      state = subsystem->state(state_name);
      break;
    }
  }
  return state;
}
}  // namespace robot
}  // namespace pvegas