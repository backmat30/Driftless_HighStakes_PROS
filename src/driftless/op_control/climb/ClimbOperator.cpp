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
  bool is_climb_state{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
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

  if (climb_voltage > 1.0) {
    if (avg_position > 5.0 || isArmInClimbState()) {
      pullBackClimber();
    }
  } else if (climb_voltage < -1.0) {
    pushForwardClimber();
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

  bool is_climbing{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemState::CLIMB_IS_CLIMBING))};

  EControllerAnalog climb_voltage_control{
      profile->getAnalogControlMapping(EControl::CLIMB_CHANGE_HEIGHT)};
  EControllerDigital stilt_control{
      profile->getDigitalControlMapping(EControl::CLIMB_TOGGLE)};
  EControllerDigital passive_control{
      profile->getDigitalControlMapping(EControl::CLIMB_TOGGLE_PASSIVES)};

  if (m_controller->getNewDigital(stilt_control)) {
    updateStiltState();
    toggleDriveTrainClimbMode();
    toggleIntakeClimbMode();
  }

  if (m_controller->getNewDigital(passive_control)) {
    if (arePassivesOut()) {
      pullInPassiveHooks();
    } else if (is_climbing) {
      pushOutPassiveHooks();
    }
  }

  double climb_voltage{m_controller->getAnalog(climb_voltage_control) *
                       VOLTAGE_CONVERSION};

  setClimberState(climb_voltage);
  climbDriveTrain(climb_voltage);

  if (arePassivesOut()) {
    m_controller->rumble(".");
  }
}

}  // namespace driftless::op_control::climb