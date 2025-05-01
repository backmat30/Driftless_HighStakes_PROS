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

void ClimbOperator::pushOutPassiveHooks() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemCommand::CLIMB_PUSH_OUT_PASSIVE_HOOKS);
}

void ClimbOperator::pullInPassiveHooks() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemCommand::CLIMB_PULL_IN_PASSIVE_HOOKS);
}

bool ClimbOperator::arePassivesOut() {
  bool are_passives_out{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemState::CLIMB_ARE_PASSIVES_OUT))};
  return are_passives_out;
}

bool ClimbOperator::isArmInClimbState() {
  bool is_climb_state{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemState::ARM_IS_CLIMB))};
  return is_climb_state;
}

double ClimbOperator::getDriveTrainLeftMotorPosition() {
  double position{*static_cast<double*>(m_robot->getState(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemState::DRIVETRAIN_GET_LEFT_POSITION))};
  return position;
}

double ClimbOperator::getDriveTrainRightMotorPosition() {
  double position{*static_cast<double*>(m_robot->getState(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemState::DRIVETRAIN_GET_RIGHT_POSITION))};
  return position;
}

void ClimbOperator::toggleDriveTrainClimbMode() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_TOGGLE_CLIMB_MODE);
}

void ClimbOperator::toggleIntakeClimbMode() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_TOGGLE_SECONDARY_PISTONS);
}

void ClimbOperator::climbDriveTrain(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::DRIVETRAIN,
                       robot::subsystems::ESubsystemCommand::DRIVETRAIN_CLIMB,
                       voltage);
}

void ClimbOperator::setClimberState(double climb_voltage) {
  double left_position{getDriveTrainLeftMotorPosition()};
  double right_position{getDriveTrainRightMotorPosition()};
  double avg_position{(left_position + right_position) / 2.0};

  if (avg_position <= 0.0) {
    pushOutPassiveHooks();
  }
  if (climb_voltage > 1.0) {
    if (avg_position > 5.0 || isArmInClimbState()) {
      pullBackClimber();
    }
    pushOutPassiveHooks();
  } else if (climb_voltage < -1.0) {
    pushForwardClimber();
    if (avg_position < 41.0) {
      pullInPassiveHooks();
    }
  }
}

ClimbOperator::ClimbOperator(
    const std::shared_ptr<driftless::io::IController>& controller,
    const std::shared_ptr<driftless::robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ClimbOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  if (!m_controller) {
    return;
  }

  EControllerAnalog climb_voltage_control{
      profile->getAnalogControlMapping(EControl::CLIMB_CHANGE_HEIGHT)};
  EControllerDigital stilt_control{
      profile->getDigitalControlMapping(EControl::CLIMB_TOGGLE)};

  if (m_controller->getNewDigital(stilt_control)) {
    updateStiltState();
    toggleDriveTrainClimbMode();
    toggleIntakeClimbMode();
  }

  double climb_voltage{m_controller->getAnalog(climb_voltage_control) *
                       VOLTAGE_CONVERSION};

  setClimberState(climb_voltage);
  if (climb_voltage <= 0.0 || arePassivesOut()) {
    climbDriveTrain(climb_voltage);
  }
}

}  // namespace driftless::op_control::climb