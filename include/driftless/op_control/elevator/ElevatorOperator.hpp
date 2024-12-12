#ifndef __ELEVATOR_OPERATOR_HPP__
#define __ELEVATOR_OPERATOR_HPP__

#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/io/RGBValue.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/elevator/EElevatorControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace op_control {
namespace elevator {
class ElevatorOperator {
 private:
  // name of the blue alliance
  static constexpr char BLUE_ALLIANCE_NAME[]{"BLUE"};

  // name of the red alliance
  static constexpr char RED_ALLIANCE_NAME[]{"RED"};

  /// @brief the name of the elevator subsystem
  static constexpr char ELEVATOR_SUBSYSTEM_NAME[]{"ELEVATOR"};

  /// @brief The name of the ring sort subsystem
  static constexpr char RING_SORT_SUBSYSTEM_NAME[]{"RING SORT"};

  // COMMAND NAMES

  // command to set the position of the elevator
  static constexpr char SET_POSITION_COMMAND_NAME[]{"SET POSITION"};

  // command to set the voltage of the elevator motors
  static constexpr char SET_VOLTAGE_COMMAND_NAME[]{"SET VOLTAGE"};

  // command to deploy the ring rejector
  static constexpr char DEPLOY_REJECTOR_COMMAND_NAME[]{"DEPLOY REJECTOR"};

  // command to retract the ring rejector
  static constexpr char RETRACT_REJECTOR_COMMAND_NAME[]{"RETRACT REJECTOR"};

  // STATE NAMES

  // position of the elevator
  static constexpr char GET_POSITION_STATE_NAME[]{"GET POSITION"};

  // The hue from the ring sorter
  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  // The rgb value from the ring sorter
  static constexpr char GET_RGB_STATE_NAME[]{"GET RGB"};

  // whether the ring sorter senses a ring
  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  // whether the ring rejector is deployed
  static constexpr char IS_DEPLOYED_STATE_NAME[]{"IS DEPLOYED"};

  /// @brief state name for retrieving the distance from the sensor to the end
  /// of the elevator
  static constexpr char GET_SENSOR_DISTANCE_TO_END_STATE_NAME[]{
      "GET DISTANCE TO END"};

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