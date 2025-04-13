#include "driftless/op_control/climb/ClimbOperator.hpp"

namespace driftless::op_control::climb {
void ClimbOperator::updateStiltState() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemCommand::CLIMB_TOGGLE_CLIMBING);
}

void ClimbOperator::pullBackClimber() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemCommand::CLIMB_PULL_BACK_CLIMBER);
}

void ClimbOperator::pushForwardClimber() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemCommand::CLIMB_PUSH_FORWARD_CLIMBER);
}

void ClimbOperator::climbDriveTrain(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_CLIMB, voltage);
}

void ClimbOperator::toggleDriveClimbMode() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_TOGGLE_CLIMB_MODE);
}

ClimbOperator::ClimbOperator(
    const std::shared_ptr<driftless::io::IController>& controller,
    const std::shared_ptr<driftless::robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ClimbOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  if(!m_controller) {
    return;
  }

  EControllerAnalog climb_voltage_control{profile->getAnalogControlMapping(EControl::CLIMB_CHANGE_HEIGHT)};
  EControllerDigital stilt_control{profile->getDigitalControlMapping(EControl::CLIMB_TOGGLE)};
  
  if(m_controller->getNewDigital(stilt_control)) {
    updateStiltState();
    toggleDriveClimbMode();
  }

  double climb_voltage_scalar{m_controller->getAnalog(climb_voltage_control)};
  if(climb_voltage_scalar > 0.25 * 127) {
    pullBackClimber();
  } else if (climb_voltage_scalar < -0.25 * 127) {
    pushForwardClimber();
  }

  climbDriveTrain(climb_voltage_scalar * 12.0);
}

  
}