#ifndef __ELEVATOR_OPERATOR_HPP__
#define __ELEVATOR_OPERATOR_HPP__

#include <memory>

#include "pvegas/io/IController.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/robot/Robot.hpp"
namespace pvegas {
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

  // the controller being used
  std::shared_ptr<pvegas::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<pvegas::robot::Robot> m_robot{};

  // updates the voltage of the elevator motors
  void updateElevatorVoltage(double voltage);

 public:
  // constructor
  ElevatorOperator(const std::shared_ptr<pvegas::io::IController>& controller,
                   const std::shared_ptr<pvegas::robot::Robot>& robot);

  // spin the elevator
  void setElevatorVoltage();
};
}  // namespace elevator
}  // namespace op_control
}  // namespace pvegas
#endif