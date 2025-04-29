#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include "driftless/alliance/EAlliance.hpp"
#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/arm/EArmControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/processes/ProcessSystem.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for arm control
/// @author Matthew Backman
namespace arm {

/// @brief Class to represent arm control
/// @author Matthew Backman
class ArmOperator {
 private:
  // the controller used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  std::shared_ptr<driftless::processes::ProcessSystem> m_process_system{};

  /// @brief determines if the robot has an alliance ring loaded
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  /// @return __bool__ True if there is an alliance ring, false otherwise
  bool hasAllianceRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  /// @depricated: Use color sort process
  /// @brief determines if the robot has an opposing alliance ring loaded
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  bool hasOpposingRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  /// @brief determines if the robot has a ring loaded
  /// @return __bool__ True if there is a ring, false otherwise
  bool hasRing();

  /// @brief Updates the arm using seperate buttons for each location
  /// @param neutral __EControllerDigital__ Button for neutral position
  /// @param load __EControllerDigital__ Button for load position
  /// @param ready __EControllerDigital__ Button for ready position
  /// @param score __EControllerDigital__ Button for score position
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  void updateSplitToggle(EControllerDigital neutral, EControllerDigital load,
                         EControllerDigital ready, EControllerDigital score,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

  /// @brief Updates the arm using a single primary button for position cycling
  /// @param toggle __EControllerDigital__ Button for primary position cycle
  /// @param rush __EControllerDigital__ Button to go to the rush position
  /// @param calibrate __EControllerDigital__ Button to calibrate the arm
  /// @param alliance_stake __EControllerDigital__ Button to go to the alliance
  /// stake position
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  void updateSmartToggle(EControllerDigital toggle, EControllerDigital rush,
                         EControllerDigital calibrate,
                         EControllerDigital alliance_stake,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

 public:
  /// @brief Constructs a new ArmOperator object
  /// @param controller __const std::shared_ptr<io::IController>&__
  /// The controller used
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot
  /// being controlled
  ArmOperator(const std::shared_ptr<driftless::io::IController>& controller,
              const std::shared_ptr<driftless::robot::Robot>& robot,
              const std::shared_ptr<driftless::processes::ProcessSystem>&
                  process_system);

  /// @brief Updates the arm
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile
  /// used for control mapping
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const std::shared_ptr<alliance::IAlliance>& alliance);
};
}  // namespace arm
}  // namespace op_control
}  // namespace driftless
#endif