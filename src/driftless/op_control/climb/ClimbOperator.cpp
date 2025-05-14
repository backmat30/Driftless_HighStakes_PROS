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
  if (voltage < 0) {
    voltage *= 8.0 / 12.0;
  }
  m_robot->sendCommand(robot::subsystems::ESubsystem::DRIVETRAIN,
                       robot::subsystems::ESubsystemCommand::DRIVETRAIN_CLIMB,
                       voltage);
}

void ClimbOperator::setClimberState(double climb_voltage) {
  double left_position{getDriveTrainLeftMotorPosition()};
  double right_position{getDriveTrainRightMotorPosition()};
  double avg_position{(left_position + right_position) / 2.0};

  bool is_climbing{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemState::CLIMB_IS_CLIMBING))};

  if (climb_voltage > 1.0) {
    if (avg_position > 5.0 || isArmInClimbState()) {
      pullBackClimber();
    }
  } else if (climb_voltage < -1.0) {
    pushForwardClimber();
    if (avg_position < 40.0 && avg_position > 10.0 && arePassivesOut()) {
      pullInPassiveHooks();

      if (is_climbing) {
        m_robot->sendCommand(
            robot::subsystems::ESubsystem::ARM,
            robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
      }
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

  bool is_climbing{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::CLIMB,
      robot::subsystems::ESubsystemState::CLIMB_IS_CLIMBING))};

  bool is_tank_drive{profile->getControlMode(EControlType::DRIVE) == 0};
  bool is_eric_robot{profile->getName() == "ERIC"};

  EControllerAnalog climb_voltage_control{
      profile->getAnalogControlMapping(EControl::CLIMB_CHANGE_HEIGHT)};
  EControllerAnalog climb_slow_voltage_control{
      profile->getAnalogControlMapping(EControl::CLIMB_CHANGE_HEIGHT_SLOW)};
  EControllerDigital stilt_control{
      profile->getDigitalControlMapping(EControl::CLIMB_TOGGLE)};
  // REALLY SCUFFED I KNOW BUT I DONT HAVE TIME TO DO IT RIGHT :(
  if ((is_eric_robot && is_tank_drive) || (!is_eric_robot && !is_tank_drive)) {
    stilt_control = profile->getDigitalControlMapping(EControl::ARM_CALIBRATE);
  }
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
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ARM,
          robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
    } else if (is_climbing) {
      pushOutPassiveHooks();
      if (is_tank_drive && is_eric_robot) {
        m_robot->sendCommand(
            robot::subsystems::ESubsystem::ARM,
            robot::subsystems::ESubsystemCommand::ARM_GO_CLIMB_READY);
      }
    }
  }
  double climb_power{m_controller->getAnalog(climb_voltage_control)};
  if (profile->getControlMode(EControlType::CLIMB) ==
      (int)EClimbControlMode::EXPONENTIAL) {
    climb_power =
        climb_power * std::abs(std::pow(climb_power, 2)) / std::pow(127, 3);
  }
  double climb_slow_power{m_controller->getAnalog(climb_slow_voltage_control)};

  if (std::abs(climb_power) < 10 && std::abs(climb_slow_power) > 10) {
    climbDriveTrain(climb_slow_power * SLOW_VOLTAGE_CONVERSION);
  } else {
    double climb_voltage{m_controller->getAnalog(climb_voltage_control) *
                         VOLTAGE_CONVERSION};

    setClimberState(climb_voltage);
    climbDriveTrain(climb_voltage);
  }

  if (arePassivesOut()) {
    m_controller->rumble(".");
  }
}

}  // namespace driftless::op_control::climb