#include "driftless/robot/subsystems/elevator/ElevatorSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
ElevatorSubsystem::ElevatorSubsystem(std::unique_ptr<IElevator>& elevator)
    : ASubsystem{SUBSYSTEM_NAME}, m_elevator{std::move(elevator)} {}

void ElevatorSubsystem::init() { m_elevator->init(); }

void ElevatorSubsystem::run() { m_elevator->run(); }

void ElevatorSubsystem::command(std::string command_name, va_list& args) {
  if (command_name == SET_POSITION_COMMAND_NAME) {
    double position{va_arg(args, double)};
    m_elevator->setPosition(position);
  } else if (command_name == SET_VOLTAGE_COMMAND_NAME) {
    double voltage{va_arg(args, double)};
    m_elevator->setVoltage(voltage);
  }
}

void* ElevatorSubsystem::state(std::string state_name) {
  void* result{nullptr};
  if (state_name == GET_POSITION_STATE_NAME) {
    result = new double{m_elevator->getPosition()};
  }

  return result;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless