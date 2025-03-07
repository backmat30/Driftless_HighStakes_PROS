#include "driftless/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
DrivetrainSubsystem::DrivetrainSubsystem(
    std::unique_ptr<IDrivetrain>& drivetrain)
    : ASubsystem{ESubsystem::DRIVETRAIN}, m_drivetrain(std::move(drivetrain)) {}

void DrivetrainSubsystem::init() { m_drivetrain->init(); }

void DrivetrainSubsystem::run() { m_drivetrain->run(); }

void DrivetrainSubsystem::command(ESubsystemCommand command_name,
                                  va_list& args) {
  if (command_name == ESubsystemCommand::DRIVETRAIN_SET_VELOCITY) {
    double left_velocity{va_arg(args, double)};
    double right_velocity{va_arg(args, double)};
    Velocity velocity{left_velocity, right_velocity};
    m_drivetrain->setVelocity(velocity);
  } else if (command_name == ESubsystemCommand::DRIVETRAIN_SET_VOLTAGE) {
    double left_voltage{va_arg(args, double)};
    double right_voltage{va_arg(args, double)};
    m_drivetrain->setVoltage(left_voltage, right_voltage);
  }
}

void* DrivetrainSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  if (state_name == ESubsystemState::DRIVETRAIN_GET_VELOCITY) {
    Velocity* velocity{new Velocity{m_drivetrain->getVelocity()}};
    result = velocity;
  } else if (state_name == ESubsystemState::DRIVETRAIN_GET_RADIUS) {
    double* radius{new double{m_drivetrain->getDriveRadius()}};
    result = radius;
  }
  return result;
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless