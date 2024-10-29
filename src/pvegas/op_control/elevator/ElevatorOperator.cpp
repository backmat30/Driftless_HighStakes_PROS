#include "pvegas/op_control/elevator/ElevatorOperator.hpp"

namespace pvegas {
namespace op_control {
namespace elevator {
void ElevatorOperator::updateElevatorVoltage(double voltage) {
  if (m_robot) {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, SET_VOLTAGE_COMMAND_NAME,
                         voltage);
  }
}

ElevatorOperator::ElevatorOperator(
    const std::shared_ptr<pvegas::io::IController>& controller,
    const std::shared_ptr<pvegas::robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ElevatorOperator::spinElevator() {
  if (!m_controller) {
    updateElevatorVoltage(0);
    return;
  }
  double voltage{
      m_controller->getDigital(EControllerDigital::TRIGGER_RIGHT_TOP) * 127.0 *
      VOLTAGE_CONVERSION};
      
  updateElevatorVoltage(voltage);
}
}  // namespace elevator
}  // namespace op_control
}  // namespace pvegas