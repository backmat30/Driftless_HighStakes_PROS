#ifndef __ELEVATOR_OPERATOR_HPP__
#define __ELEVATOR_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/elevator/EElevatorControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace op_control {
namespace elevator {
class ElevatorOperator {
 private:
  // name of the elevator subsystem
  static constexpr char ELEVATOR_SUBSYSTEM_NAME[]{"ELEVATOR"};

  // name of the command to set the voltage
  static constexpr char SET_VOLTAGE_COMMAND_NAME[]{"SET VOLTAGE"};

  // conversion factor to voltage
  static constexpr double VOLTAGE_CONVERSION{12.0 / 127.0};

  // voltage used to run the elevator by default
  static constexpr double DEFAULT_VOLTAGE{127.0};

  // the controller being used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // updates the voltage of the elevator motors
  void updateElevatorVoltage(double voltage);

 public:
  // constructor
  ElevatorOperator(const std::shared_ptr<driftless::io::IController>& controller,
                   const std::shared_ptr<driftless::robot::Robot>& robot);

  // spin the elevator
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile);
};
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless
#endif