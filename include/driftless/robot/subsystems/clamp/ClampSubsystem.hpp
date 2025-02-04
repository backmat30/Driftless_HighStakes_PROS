#ifndef __CLAMP_SUBSYSTEM_HPP__
#define __CLAMP_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/clamp/IClamp.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the clamp subsystem
/// @author Matthew Backman
namespace clamp {

/// @brief Subsystem class for managing a clamp
/// @author Matthew Backman
class ClampSubsystem : public ASubsystem {
 private:
  // the clamp used by the subsystem
  std::unique_ptr<IClamp> m_clamp{};

 public:
  /// @brief Constructs a new clamp subsystem object
  /// @param clamp __std::unique_ptr<IClamp>&__ The clamp to use
  ClampSubsystem(std::unique_ptr<IClamp>& clamp);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Executes a given command
  /// @param command_name __ESubsystemCommand__ The command to execute
  /// @param args __va_list&__ The arguments for the command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets a state of the subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state
  void* state(ESubsystemState state_name) override;
};

}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif