#ifndef __CONTROL_SYSTEM_HPP__
#define __CONTROL_SYSTEM_HPP__

#include <cstdarg>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driftless/control/AControl.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Class to handle control algorithm management
/// @author Matthew Backman
class ControlSystem {
 private:
  // possible controls for the robot
  std::map<EControl, std::unique_ptr<AControl>> controls{};

  // the name of the active control
  EControl active_control{};

 public:
  /// @brief Adds a control to the control system
  /// @param control __std::unique_ptr<AControl>&__ The control added
  void addControl(std::unique_ptr<AControl> &control);

  /// @brief Removes a control from the system
  /// @param control __EControl__ The control to remove
  /// @return __bool__ True if removed, false if not found
  bool removeControl(EControl control);

  /// @brief Pauses the active control
  void pause();

  /// @brief Resumes the active control
  void resume();

  /// @brief Initializes all controls in the control system
  void init();

  /// @brief Runs all controls in the control system
  void run();

  /// @brief Sends a command to a given control
  /// @param control_name __EControl__ The control to command
  /// @param command_name __EControlCommand__ The command given
  /// @param ... __va_list__ Potential arguements for the command
  void sendCommand(EControl control_name, EControlCommand command_name, ...);

  /// @brief Gets a state of a given control
  /// @param control_name __EControl__ The control to get a state from
  /// @param state_name __EControlState__ The state to get
  /// @return __void*__ The state of the control
  void *getState(EControl control_name, EControlState state_name);
};
}  // namespace control
}  // namespace driftless
#endif