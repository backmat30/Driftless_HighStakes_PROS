#ifndef __CONTROL_SYSTEM_HPP__
#define __CONTROL_SYSTEM_HPP__

#include <cstdarg>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driftless/control/AControl.hpp"
namespace driftless {
namespace control {
class ControlSystem {
 private:
  // possible controls for the robot
  std::map<EControl, std::unique_ptr<AControl>> controls{};

  // the name of the active control
  EControl active_control{};

 public:
  // adds a control type to the robot
  void addControl(std::unique_ptr<AControl> &control);

  // removes a control type
  bool removeControl(EControl control);

  // pauses the control system
  void pause();

  // resumes the control system
  void resume();

  // init ALL controls within the system
  void init();

  // runs ALL controls within the system
  void run();

  // sends a command to a specified control
  void sendCommand(EControl control_name, EControlCommand command_name, ...);

  // gets a state of a specified control
  void *getState(EControl control_name, EControlState state_name);
};
}  // namespace control
}  // namespace driftless
#endif