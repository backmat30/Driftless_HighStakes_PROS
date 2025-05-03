#include "driftless/op_control/elevator/ElevatorOperator.hpp"

namespace driftless {
namespace op_control {
namespace elevator {
void ElevatorOperator::updateElevatorVoltage(double voltage) {
  if (m_robot) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::ELEVATOR,
        robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
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
    EControllerDigital toggle,
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  bool toggle_color_sort{m_controller->getNewDigital(toggle)};
  if (toggle_color_sort) {
    color_sort_active = !color_sort_active;
  }

  if (color_sort_active) {
    void* has_ring_state{m_robot->getState(
        robot::subsystems::ESubsystem::RING_SORT,
        robot::subsystems::ESubsystemState::RING_SORT_HAS_RING)};
    bool has_ring{has_ring_state != nullptr &&
                  *static_cast<bool*>(has_ring_state)};

    void* ring_rgb_state{m_robot->getState(
        robot::subsystems::ESubsystem::RING_SORT,
        robot::subsystems::ESubsystemState::RING_SORT_GET_RGB)};
    io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

    void* position_state{m_robot->getState(
        robot::subsystems::ESubsystem::ELEVATOR,
        robot::subsystems::ESubsystemState::ELEVATOR_GET_POSITION)};
    double position{*static_cast<double*>(position_state)};

    void* distance_to_end_state{m_robot->getState(
        robot::subsystems::ESubsystem::RING_SORT,
        robot::subsystems::ESubsystemState::RING_SORT_GET_DISTANCE_TO_END)};
    double distance_to_end{*static_cast<double*>(distance_to_end_state)};

    if (has_ring) {
      if ((alliance->getAlliance() == alliance::EAlliance::BLUE &&
           ring_rgb.red >= ring_rgb.blue) ||
          (alliance->getAlliance() == alliance::EAlliance::RED &&
           ring_rgb.blue >= ring_rgb.red)) {
        latest_ring_pos = position;
      }
    }

    if (position <= latest_ring_pos + distance_to_end &&
        position >= latest_ring_pos - 0.1) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ELEVATOR,
          robot::subsystems::ESubsystemCommand::ELEVATOR_DEPLOY_REJECTOR);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ELEVATOR,
          robot::subsystems::ESubsystemCommand::ELEVATOR_RETRACT_REJECTOR);
      latest_ring_pos = -__DBL_MAX__;
    }
  } else {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::ELEVATOR,
        robot::subsystems::ESubsystemCommand::ELEVATOR_RETRACT_REJECTOR);
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

  bool is_climbing{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemState::CLIMB_IS_CLIMBING))};

  if (!is_climbing) {
    switch (static_cast<EElevatorControlMode>(
        profile->getControlMode(EControlType::ELEVATOR))) {
      case EElevatorControlMode::HOLD:
        updateHold(spin, reverse);
        break;
      case EElevatorControlMode::TOGGLE:
        updateToggle(spin, reverse);
        break;
    }
  }
  // moved to color sort process
  // updateRingSensor(toggle_color_sort, alliance);
}
}  // namespace elevator
}  // namespace op_control
}  // namespace driftless