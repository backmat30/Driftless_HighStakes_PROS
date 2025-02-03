#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

#include "pros/screen.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The robot class
/// @author Matthew Backman
class Robot {
 private:
  std::map<subsystems::ESubsystem, std::unique_ptr<subsystems::ASubsystem>> subsystems{};

 public:
  /// @brief Adds a subsystem to the robot
  /// @param subsystem __unique_ptr<subsystems::ASubsystem>&__ A unique pointer to the subsystem to add
  void addSubsystem(std::unique_ptr<subsystems::ASubsystem> &subsystem);

  /// @brief Removes a subsystem from the robot
  /// @param subsystem __subsystems::ESubsystem__ The subsystem to remove
  /// @return __bool__ True if the subsystem was removed, false otherwise
  bool removeSubsystem(subsystems::ESubsystem subsystem);

  /// @brief Initializes the robot
  void init();

  /// @brief Runs the robot
  void run();

  /// @brief Sends a command to a subsystem
  /// @param subsystem_name __subsystems::ESubsystem__ The subsystem to send the command to
  /// @param command_name __subsystems::ESubsystemCommand__ The command to send
  /// @param ... __va_list__ The arguments for the command
  void sendCommand(subsystems::ESubsystem subsystem_name,
                   subsystems::ESubsystemCommand command_name, ...);

  /// @brief Gets the state of a subsystem
  /// @param subsystem_name __subsystems::ESubsystem__ The subsystem to get the state of
  /// @param state_name __subsystems::ESubsystemState__ The state to get
  /// @return __void*__ A pointer to the state
  void *getState(subsystems::ESubsystem subsystem_name,
                 subsystems::ESubsystemState state_name);
};
}  // namespace robot
}  // namespace driftless
#endif