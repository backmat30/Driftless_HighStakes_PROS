#ifndef __INTAKE_SUBSYSTEM_HPP__
#define __INTAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/intake/IHeightControl.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for intake subsystem code
/// @author Matthew Backman
namespace intake {

/// @brief Subsystem class for managing the intake mechanism
/// @author Matthew Backman
class IntakeSubsystem : public ASubsystem {
 private:
  // the intake object
  std::unique_ptr<IIntake> m_intake{};

  // the intake height controller
  std::unique_ptr<IHeightControl> m_height_control{};

 public:
  /// @brief Constructs a new intake subsystem
  /// @param intake __std::unique_ptr<IIntake>&__ The intake object
  /// @param height_control __std::unique_ptr<IHeightControl>&__ The height control object
  IntakeSubsystem(std::unique_ptr<IIntake>& intake,
                  std::unique_ptr<IHeightControl>& height_control);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name __ESubsystemCommand__ The command to send
  /// @param args __va_list&__ The arguments for the command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets a state of the subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state
  void* state(ESubsystemState state_name) override;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif