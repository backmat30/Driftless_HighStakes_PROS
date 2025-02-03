#ifndef __A_SUBSYSTEM_HPP__
#define __A_SUBSYSTEM_HPP__

#include <cstdarg>
#include <string>

#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The abstract class for subsystems
/// @author Matthew Backman
class ASubsystem {
 private:
  ESubsystem m_name{};

 public:
  /// @brief Default constructor
  ASubsystem() = default;

  /// @brief Copy constructor
  /// @param other __const ASubsystem&__ The subsystem to copy
  ASubsystem(const ASubsystem& other) = default;

  /// @brief Constructor
  /// @param new_name __ESubsystem__ The name of the subsystem
  ASubsystem(ESubsystem new_name) : m_name{new_name} {}

  /// @brief Destructor
  virtual ~ASubsystem() = default;

  /// @brief Gets the name of the subsystem
  /// @return __ESubsystem__ The name of the subsystem
  const ESubsystem& getName() const { return m_name; }

  /// @brief Initializes the subsystem
  virtual void init() = 0;

  /// @brief Runs the subsystem
  virtual void run() = 0;

  /// @brief Sends a command to the subsystem
  /// @param command_name __ESubsystemCommand__ The command to send
  /// @param args __va_list__ The arguments for the command
  virtual void command(ESubsystemCommand command_name, va_list& args) = 0;

  /// @brief Gets the state of the subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state
  virtual void* state(ESubsystemState state_name) = 0;

  /// @brief Assignment operator
  /// @param rhs __const ASubsystem&__ The subsystem to assign
  /// @return __ASubsystem&__ The assigned subsystem
  ASubsystem& operator=(const ASubsystem& rhs) = default;
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif