#include "pvegas/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
IntakeSubsystem::IntakeSubsystem(
    std::unique_ptr<IIntake>& intake,
    std::unique_ptr<IHeightControl>& height_control)
    : ASubsystem{SUBSYSTEM_NAME},
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

void IntakeSubsystem::command(std::string command_name, va_list& args) {
  if (command_name == SPIN_INTAKE_COMMAND_NAME) {
    double voltage{va_arg(args, double)};
    m_intake->setVoltage(voltage);
  } else if (command_name == SET_HEIGHT_COMMAND_NAME) {
    bool raised{static_cast<bool>(va_arg(args, int))};
    m_height_control->setHeight(raised);
  }
}

void* IntakeSubsystem::state(std::string state_name) {
  void* result{nullptr};
  if (state_name == GET_HEIGHT_STATE_NAME) {
    result = new bool{m_height_control->isRaised()};
  }
  return result;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas