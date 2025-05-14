#include "driftless/robot/subsystems/elevator/ElevatorSubsystem.hpp"
#include "pros/screen.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
ElevatorSubsystem::ElevatorSubsystem(
    std::unique_ptr<IElevator>& elevator,
    std::unique_ptr<IRingRejection>& ring_rejector)
    : ASubsystem{ESubsystem::ELEVATOR},
      m_elevator{std::move(elevator)},
      m_ring_rejector{std::move(ring_rejector)} {}

void ElevatorSubsystem::init() {
  m_elevator->init();
  m_ring_rejector->init();
}

void ElevatorSubsystem::run() {
  m_elevator->run();
  m_ring_rejector->run();
}

void ElevatorSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  if (command_name == ESubsystemCommand::ELEVATOR_SET_POSITION) {
    double position{va_arg(args, double)};
    m_elevator->setPosition(position);
  } else if (command_name == ESubsystemCommand::ELEVATOR_SET_VOLTAGE) {
    double voltage{va_arg(args, double)};
    m_elevator->setVoltage(voltage);
  } else if (command_name == ESubsystemCommand::ELEVATOR_DEPLOY_REJECTOR) {
    m_ring_rejector->deploy();
  } else if (command_name == ESubsystemCommand::ELEVATOR_RETRACT_REJECTOR) {
    m_ring_rejector->retract();
  } else if (command_name == ESubsystemCommand::ELEVATOR_REJECT_LEFT) {
    m_ring_rejector->setDeploymentDirection(ERejectionDirection::LEFT);
  } else if (command_name == ESubsystemCommand::ELEVATOR_REJECT_RIGHT) {
    m_ring_rejector->setDeploymentDirection(ERejectionDirection::RIGHT);
  } else if (command_name == ESubsystemCommand::ELEVATOR_PAUSE) {
    m_elevator->pause();
  } else if (command_name == ESubsystemCommand::ELEVATOR_RESUME) {
    m_elevator->resume();
  }
}

void* ElevatorSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  if (state_name == ESubsystemState::ELEVATOR_GET_POSITION) {
    result = new double{m_elevator->getPosition()};
  } else if (state_name == ESubsystemState::ELEVATOR_IS_DEPLOYED) {
    result = new bool{m_ring_rejector->isDeployed()};
  } else if (state_name == ESubsystemState::ELEVATOR_IS_PAUSED) {
    result = new bool{m_elevator->isPaused()};
  }

  return result;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless