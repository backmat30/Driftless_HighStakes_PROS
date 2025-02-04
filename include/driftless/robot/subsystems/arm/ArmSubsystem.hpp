#ifndef __ARM_SUBSYSTEM_HPP__
#define __ARM_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/arm/IArmMotion.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for arm subsystem code
/// @author Matthew Backman
namespace arm {

/// @brief The class for the arm subsystem
/// @author Matthew Backman
class ArmSubsystem : public ASubsystem {
 private:
  // the arm motion controller
  std::unique_ptr<IArmMotion> m_arm_motion{};

 public:
  /// @brief Constructs a new arm subsystem
  /// @param arm_motion The arm motion controller
  ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name The command to send
  /// @param args The arguments for the command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets a state of the subsystem
  /// @param state_name The state to get
  /// @return The state
  void* state(ESubsystemState state_name) override;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif