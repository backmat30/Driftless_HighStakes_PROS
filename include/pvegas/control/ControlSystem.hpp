#ifndef __CONTROL_SYSTEM_HPP__
#define __CONTROL_SYSTEM_HPP__

#include <cstdarg>
#include <memory>
#include <string>
#include <vector>

#include "pvegas/control/AControl.hpp"
namespace pvegas {
namespace control {
class ControlSystem {
 private:
  // possible controls for the robot
  std::vector<std::unique_ptr<AControl>> controls{};

  // the name of the active control
  std::string active_control{};

 public:
  // adds a control type to the robot
  void addControl(std::unique_ptr<AControl> &control);

  // removes a control type
  bool removeControl(std::string control);

  // pauses the control system
  void pause();

  // resumes the control system
  void resume();

  // init ALL controls within the system
  void init();

  // runs ALL controls within the system
  void run();

  // sends a command to a specified control
  void sendCommand(std::string control_name, std::string command_name, ...);

  // gets a state of a specified control
  void *getState(std::string control_name, std::string state_name);
};
}  // namespace control
}  // namespace pvegas
#endif