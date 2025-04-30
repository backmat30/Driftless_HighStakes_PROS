#ifndef __INTAKE_OPERATOR_HPP__
#define __INTAKE_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/intake/EIntakeControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for intake control
/// @author Matthew Backman
namespace intake {

/// @brief Class to represent intake control
/// @author Matthew Backman
class IntakeOperator {
 private:
  // the controller use
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  /// @brief Updates the intake using split buttons for control
  /// @param up __EControllerDigital__ The button used to move the intake up
  /// @param down __EControllerDigital__ The button used to move the intake down
  void updateSplitToggle(EControllerDigital up, EControllerDigital down);

  /// @brief Updates the intake using a single button to toggle states
  /// @param toggle __EControllerDigital__ The button used to toggle states
  void updateSingleToggle(EControllerDigital toggle);

  /// @brief Updates the intake using a held button
  /// @param up __EControllerDigital__ The button used to raise the intake
  void updateHoldUp(EControllerDigital up);

  /// @brief Updates the intake spinners
  /// @param spin __EControllerDigital__ The button used to spin the wheels
  /// forward
  /// @param reverse __ECOntrollerDigital__ The button used to spin the wheels
  /// reverse
  void updateSpinner(EControllerDigital spin, EControllerDigital reverse);

  /// @brief Updates the secondary pistons of the intake
  /// @param toggle __EControllerDigital__ The button used to toggle the pistons
  void updateSecondaryPistons(EControllerDigital toggle);

 public:
  /// @brief Constructs a new IntakeOperator object
  /// @param controller __std::shared_ptr<io::IController>&__ The controller
  /// used by the robot
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  IntakeOperator(const std::shared_ptr<driftless::io::IController>& controller,
                 const std::shared_ptr<driftless::robot::Robot>& robot);

  /// @brief Updates the state of the intake
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile
  /// used for control mappings
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile);
};
}  // namespace intake
}  // namespace op_control
}  // namespace driftless
#endif