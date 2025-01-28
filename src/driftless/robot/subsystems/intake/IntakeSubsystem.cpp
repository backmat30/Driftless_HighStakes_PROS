#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
IntakeSubsystem::IntakeSubsystem(
    std::unique_ptr<IIntake>& intake,
    std::unique_ptr<IHeightControl>& height_control)
    : ASubsystem{ESubsystem::INTAKE},
      m_intake{std::move(intake)},
      m_height_control{std::move(height_control)} {}

void IntakeSubsystem::init() {
  m_intake->init();
  m_height_control->init();
}

void IntakeSubsystem::run() {
  m_intake->run();
  m_height_control->run();
}

void IntakeSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  if (command_name == ESubsystemCommand::INTAKE_SET_VOLTAGE) {
    double voltage{static_cast<double>(va_arg(args, double))};
    m_intake->setVoltage(voltage);
  } else if (command_name == ESubsystemCommand::INTAKE_SET_HEIGHT) {
    bool raised{static_cast<bool>(va_arg(args, int))};
    m_height_control->setHeight(raised);
  }
}

void* IntakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  if (state_name == ESubsystemState::INTAKE_GET_HEIGHT) {
    result = new bool{m_height_control->isRaised()};
  }
  return result;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless