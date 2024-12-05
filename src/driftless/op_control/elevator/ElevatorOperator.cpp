#include "driftless/op_control/elevator/ElevatorOperator.hpp"

namespace driftless {
namespace op_control {
namespace elevator {
void ElevatorOperator::updateElevatorVoltage(double voltage) {
  if (m_robot) {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, SET_VOLTAGE_COMMAND_NAME,
                         voltage);
  }
}

void ElevatorOperator::updateHold(EControllerDigital spin,
                                  EControllerDigital reverse) {
  bool do_spin{m_controller->getDigital(spin)};
  bool do_reverse{m_controller->getDigital(reverse)};

  if (do_spin && !do_reverse) {
    updateElevatorVoltage(DEFAULT_VOLTAGE);
  } else if (!do_spin && do_reverse) {
    updateElevatorVoltage(-DEFAULT_VOLTAGE);
  } else {
    updateElevatorVoltage(0.0);
  }
}

void ElevatorOperator::updateToggle(EControllerDigital spin,
                                    EControllerDigital reverse) {
  bool toggle_spin{m_controller->getNewDigital(spin)};
  bool toggle_reverse{m_controller->getNewDigital(reverse)};

  // TODO
}

void ElevatorOperator::updateRingSensor(
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  void* has_ring_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, HAS_RING_STATE_NAME)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, GET_RGB_STATE_NAME)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  void* position_state{
      m_robot->getState(ELEVATOR_SUBSYSTEM_NAME, GET_POSITION_STATE_NAME)};
  double position{*static_cast<double*>(position_state)};

  void* distance_to_end_state{m_robot->getState(
      RING_SORT_SUBSYSTEM_NAME, GET_SENSOR_DISTANCE_TO_END_STATE_NAME)};
  double distance_to_end{*static_cast<double*>(distance_to_end_state)};

  if (has_ring) {
    if ((alliance->getName() == BLUE_ALLIANCE_NAME &&
         ring_rgb.red >= ring_rgb.blue) ||
        (alliance->getName() == RED_ALLIANCE_NAME &&
         ring_rgb.blue >= ring_rgb.red)) {
      latest_ring_pos = position;
    }
  }

  if (position <= latest_ring_pos + distance_to_end &&
      position >= latest_ring_pos - 0.1) {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, DEPLOY_REJECTOR_COMMAND_NAME);
  } else {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME,
                         RETRACT_REJECTOR_COMMAND_NAME);
    latest_ring_pos = -__DBL_MAX__;
  }
}

ElevatorOperator::ElevatorOperator(
    const std::shared_ptr<driftless::io::IController>& controller,
    const std::shared_ptr<driftless::robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ElevatorOperator::update(
    const std::unique_ptr<driftless::profiles::IProfile>& profile,
    const std::shared_ptr<driftless::alliance::IAlliance>& alliance) {
  EControllerDigital spin{
      profile->getDigitalControlMapping(EControl::ELEVATOR_SPIN)};
  EControllerDigital reverse{
      profile->getDigitalControlMapping(EControl::ELEVATOR_REVERSE)};
  EControllerDigital toggle{
      profile->getDigitalControlMapping(EControl::ELEVATOR_TOGGLE)};

  if (!m_controller) {
    updateElevatorVoltage(0);
    return;
  }
  bool run{};

  switch (static_cast<EElevatorControlMode>(
      profile->getControlMode(EControlType::ELEVATOR))) {
    case EElevatorControlMode::HOLD:
      updateHold(spin, reverse);
      break;
    case EElevatorControlMode::TOGGLE:
      updateToggle(spin, reverse);
      break;
  }

  updateRingSensor(alliance);
}
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless