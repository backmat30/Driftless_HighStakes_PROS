#ifndef __ELEVATOR_OPERATOR_HPP__
#define __ELEVATOR_OPERATOR_HPP__

#include <memory>

#include "driftless/alliance/EAlliance.hpp"
#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/io/RGBValue.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/elevator/EElevatorControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @date 2024-2025
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for elevator control
/// @date 2024-2025
/// @author Matthew Backman
namespace elevator {

/// @brief Class to represent elevator control
/// @date 2024-2025
/// @details This class provides control mechanisms for the elevator subsystem.
/// @author Matthew Backman
class ElevatorOperator {
 private:
  // name of the blue alliance
  static constexpr char BLUE_ALLIANCE_NAME[]{"BLUE"};

  // name of the red alliance
  static constexpr char RED_ALLIANCE_NAME[]{"RED"};

  // voltage used to run the elevator by default
  static constexpr double DEFAULT_VOLTAGE{12.0};

  // the controller being used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  /// @brief The elevators position the last time the sensor detected an
  /// opposing ring
  double latest_ring_pos{-__DBL_MAX__};

  /// @brief Whether or not the color sort is active
  bool color_sort_active{true};

  // updates the voltage of the elevator motors
  void updateElevatorVoltage(double voltage);

  /// @brief Updates the elevator using held buttons
  /// @param spin __EControllerDigital__ The button to spin the intake forwards
  /// @param reverse __EControllerDigital__ The button to spin the intake reverse
  void updateHold(EControllerDigital spin, EControllerDigital reverse);

  /// @brief Updates the elevator using new button presses
  /// @param spin __EControllerDigital__ The button to spin the intake forwards
  /// @param reverse __EControllerDigital__ The button to spin the intake reverse
  void updateToggle(EControllerDigital spin, EControllerDigital reverse);

  /// @deprecated: Use color sort process
  /// @brief Updates the ring sensor and ring rejector
  /// @param toggle __EControllerDigital__ The button to toggle the ring sensor
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The alliance information
  void updateRingSensor(EControllerDigital toggle,
                        const std::shared_ptr<alliance::IAlliance>& alliance);

 public:
  /// @brief Constructs a new ElevatorOperator object
  /// @param controller __const std::shared_ptr<driftless::io::IController>&__ The controller used by the robot
  /// @param robot __const std::shared_ptr<driftless::robot::Robot>&__ The robot being controlled
  ElevatorOperator(
      const std::shared_ptr<driftless::io::IController>& controller,
      const std::shared_ptr<driftless::robot::Robot>& robot);

  /// @brief Updates the state of the elevator
  /// @param profile __const std::unique_ptr<driftless::profiles::IProfile>&__ The profile used for control mappings
  /// @param alliance __const std::shared_ptr<driftless::alliance::IAlliance>&__ The alliance information
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const std::shared_ptr<driftless::alliance::IAlliance>& alliance);
};
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless
#endif