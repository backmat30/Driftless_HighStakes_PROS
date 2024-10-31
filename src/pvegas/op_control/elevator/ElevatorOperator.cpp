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

void ElevatorOperator::update(
    const std::unique_ptr<pvegas::profiles::IProfile>& profile) {
  EControllerDigital spin{
      profile->getDigitalControlMapping(EControl::ELEVATOR_SPIN)};
  EControllerDigital toggle{
      profile->getDigitalControlMapping(EControl::ELEVATOR_TOGGLE)};

  if (!m_controller) {
    updateElevatorVoltage(0);
    return;
  }
  bool run{};

  switch (static_cast<EElevatorControlMode>(profile->getControlMode(EControlType::ELEVATOR))) {
    case EElevatorControlMode::HOLD:
      run = m_controller->getDigital(spin);
      break;
    case EElevatorControlMode::TOGGLE:
      if (m_controller->getNewDigital(toggle)) {
        run = !run;
      }
      break;
  }

  double voltage{run * DEFAULT_VOLTAGE * VOLTAGE_CONVERSION};
  updateElevatorVoltage(voltage);
}
}  // namespace elevator
}  // namespace op_control
}  // namespace pvegas