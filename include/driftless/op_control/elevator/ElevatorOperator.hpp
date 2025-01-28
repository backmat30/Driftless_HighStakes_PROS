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

namespace driftless {
namespace op_control {
namespace elevator {
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
  /// @param spin The button to spin the intake forwards
  /// @param reverse The button to spin the intake reverse
  void updateHold(EControllerDigital spin, EControllerDigital reverse);

  /// @brief Updates the elevator using new button presses
  /// @param spin The button to spin the intake forwards
  /// @param reverse The button to spin the intake reverse
  void updateToggle(EControllerDigital spin, EControllerDigital reverse);

  /// @deprecated: Use color sort process
  /// @brief Updates the ring sensor and ring rejector
  void updateRingSensor(EControllerDigital toggle,
                        const std::shared_ptr<alliance::IAlliance>& alliance);

 public:
  // constructor
  ElevatorOperator(
      const std::shared_ptr<driftless::io::IController>& controller,
      const std::shared_ptr<driftless::robot::Robot>& robot);

  // spin the elevator
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const std::shared_ptr<driftless::alliance::IAlliance>& alliance);
};
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless
#endif