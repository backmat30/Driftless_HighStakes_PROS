#ifndef __CLAMP_OPERATOR_HPP__
#define __CLAMP_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/clamp/EClampControlMode.hpp"
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

/// @brief Namespace for clamp control
/// @author Matthew Backman
namespace clamp {

/// @brief Class to represent clamp control
/// @author Matthew Backman
class ClampOperator {
 private:
  // the controller being used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  /// @brief Updates the clamp using a held button
  /// @param hold __EControllerDigital__ Button used to hold the clamp down
  void updateHold(EControllerDigital hold);

  /// @brief Updates the clamp using a single button to toggle
  /// @param toggle __EControllerDigital__ Button used to toggle the clamp
  /// between states
  void updateSingleToggle(EControllerDigital toggle);

  /// @brief Updates the clamp using a button for each state
  /// @param grab __EControllerDigital__ Button used to grab with the clamp
  /// @param release __EControllerDigital__ Button used to release the clamp
  void updateSplitToggle(EControllerDigital grab, EControllerDigital release);

  // update the state of the clamp
  void updateClampState(bool state);

 public:
  // construct a new ClampOperator object
  ClampOperator(const std::shared_ptr<driftless::io::IController>& controller,
                const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the clamp based on the profile specifications
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile);
};
}  // namespace clamp
}  // namespace op_control
}  // namespace driftless
#endif