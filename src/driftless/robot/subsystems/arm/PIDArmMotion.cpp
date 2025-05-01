#include "driftless/robot/subsystems/arm/PIDArmMotion.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
void PIDArmMotion::taskLoop(void* params) {
  PIDArmMotion* instance{static_cast<PIDArmMotion*>(params)};

  while (true) {
    instance->taskUpdate();
  }
}

void PIDArmMotion::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }
  uint32_t time_calibrating{m_clock->getTime() - calibrate_time};
  if (calibrating && time_calibrating >= 100 &&
      m_rotation_motors.getAngularVelocity() >= 0.0 &&
      m_linear_motors.getAngularVelocity() >= 0.0) {
    m_rotation_motors.setPosition(0.0);
    m_linear_motors.setPosition(0.0);
    calibrating = false;
    state == EState::LOAD;
    linear_target_position = m_linear_load_position;
    rotational_target_position = m_rotational_load_position;
  }
  if (!calibrating) {
    updateState();
    updatePreviousState();
    updatePosition();
  }
  if (m_mutex) {
    m_mutex->give();
  }
  m_delayer->delay(TASK_DELAY);
}

void PIDArmMotion::updateState() {
  double rotation_position{getRotationalPosition()};
  double linear_position{getLinearPosition()};

  switch (state) {
    case EState::NEUTRAL_MOTION:
      if (std::abs(rotation_position - m_rotational_neutral_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_neutral_position) <=
              m_linear_tolerance) {
        state = EState::NEUTRAL;
      }
      break;
    case EState::LOAD_MOTION:
      if (std::abs(rotation_position - m_rotational_load_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_load_position) <=
              m_linear_tolerance) {
        state = EState::LOAD;
      }
      break;
    case EState::READY_MOTION:
      if (std::abs(rotation_position - m_rotational_ready_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_ready_position) <=
              m_linear_tolerance) {
        state = EState::READY;
      }
      break;
    case EState::SCORE_MOTION:
      if ((m_rotational_score_position - rotation_position) <=
              m_rotational_tolerance &&
          (linear_position - m_linear_score_position) <= m_linear_tolerance) {
        state = EState::LOAD_MOTION;
        rotational_target_position = m_rotational_load_position;
        linear_target_position = m_linear_load_position;
      }
      break;
    case EState::RUSH_MOTION:
      if (std::abs(rotation_position - m_rotational_rush_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_rush_position) <=
              m_rotational_tolerance) {
        state = EState::RUSH;
      }
      break;
    case EState::CLIMB_READY_MOTION:
      if (std::abs(rotation_position - m_rotational_climb_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_climb_ready_position) <=
              m_linear_tolerance) {
        state = EState::CLIMB_READY;
      }
      break;
    case EState::CLIMB_MOTION:
      if (std::abs(rotation_position - m_rotational_climb_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_climb_position) <=
              m_linear_tolerance) {
        state = EState::CLIMB;
      }
      break;
    case EState::LOAD_INTERMEDIATE:
      if (previous_state != EState::NEUTRAL ||
          (std::abs(m_rotational_load_intermediate_position -
                    rotation_position) <= m_rotational_tolerance &&
           std::abs(m_linear_load_intermediate_position - linear_position) <=
               m_linear_tolerance)) {
        state = EState::LOAD_MOTION;
        rotational_target_position = m_rotational_load_position;
        linear_target_position = m_linear_load_position;
      }
      break;
    case EState::READY_INTERMEDIATE:
      if (m_rotational_rush_intermediate_position - rotation_position <=
          m_rotational_tolerance) {
        state = EState::READY_MOTION;
        rotational_target_position = m_rotational_ready_position;
        linear_target_position = m_linear_ready_position;
      }
      break;
    case EState::SCORE_INTERMEDIATE:
      if (m_rotational_score_intermediate_position - rotation_position <=
          m_rotational_tolerance) {
        state = EState::SCORE_MOTION;
        rotational_target_position = m_rotational_score_position;
        linear_target_position = m_linear_score_position;
      }
      break;
    case EState::RUSH_INTERMEDIATE:
      if (m_rotational_rush_intermediate_position - rotation_position <=
          m_rotational_tolerance) {
        state = EState::RUSH_MOTION;
        rotational_target_position = m_rotational_rush_position;
        linear_target_position = m_linear_rush_position;
      }
      break;
    case EState::ALLIANCE_STAKE_INTERMEDIATE:
      if ((m_rotational_alliance_stake_intermediate_position -
           rotation_position) <= m_rotational_tolerance) {
        state = EState::ALLIANCE_STAKE_MOTION;
        rotational_target_position = m_rotational_alliance_stake_position;
        linear_target_position = m_linear_alliance_stake_position;
      }
      break;
  }
}

void PIDArmMotion::updatePreviousState() {
  switch (state) {
    case EState::NEUTRAL:
      previous_state = EState::NEUTRAL;
      break;
    case EState::LOAD:
      previous_state = EState::LOAD;
      break;
    case EState::READY:
      previous_state = EState::READY;
      break;
    case EState::RUSH:
      previous_state = EState::RUSH;
      break;
  }
}

void PIDArmMotion::updatePosition() {
  double rotational_voltage{m_rotational_pid.getControlValue(
      getRotationalPosition(), rotational_target_position)};

  double linear_voltage{m_linear_pid.getControlValue(getLinearPosition(),
                                                     linear_target_position)};

  m_rotation_motors.setVoltage(rotational_voltage);
  m_linear_motors.setVoltage(linear_voltage);
}

double PIDArmMotion::getRotationalPosition() {
  double position{};
  if (m_rotation_sensor) {
    position = m_rotation_sensor->getAngle() * SENSOR_TO_ARM_ROTATIONS;
  } else if (m_potentiometer) {
    position = m_potentiometer->getAngle() * SENSOR_TO_ARM_ROTATIONS;
  } else {
    position = m_rotation_motors.getPosition() * MOTOR_TO_ARM_ROTATIONS;
  }

  return position;
}

double PIDArmMotion::getLinearPosition() {
  return m_linear_motors.getPosition();
}

double PIDArmMotion::getRotationalEfficiency() {
  return m_rotation_motors.getEfficiency();
}

double PIDArmMotion::getLinearEfficiency() {
  return m_linear_motors.getEfficiency();
}

void PIDArmMotion::init() {
  m_rotation_motors.init();
  m_linear_motors.init();
  m_rotational_pid.reset();
  m_linear_pid.reset();

  if (m_rotation_sensor) {
    m_rotation_sensor->init();
  }
  if (m_potentiometer) {
    m_potentiometer->init();
  }
}

void PIDArmMotion::run() { m_task->start(&PIDArmMotion::taskLoop, this); }

void PIDArmMotion::calibrate() {
  if (m_mutex) {
    m_mutex->take();
  }
  calibrating = true;
  calibrate_time = m_clock->getTime();
  m_rotation_motors.setVoltage(-12.0);
  m_linear_motors.setVoltage(-12.0);

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goNeutral() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::NEUTRAL) {
    state = EState::NEUTRAL_MOTION;
    rotational_target_position = m_rotational_neutral_position;
    linear_target_position = m_linear_neutral_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goLoad() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::LOAD && state != EState::LOAD_MOTION) {
    state = EState::LOAD_INTERMEDIATE;
    rotational_target_position = m_rotational_load_intermediate_position;
    linear_target_position = m_linear_load_intermediate_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goReady() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::READY && state != EState::READY_MOTION) {
    state = EState::READY_INTERMEDIATE;
    rotational_target_position = m_rotational_ready_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goScore() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::SCORE && state != EState::SCORE_MOTION) {
    state = EState::SCORE_INTERMEDIATE;
    rotational_target_position = m_rotational_score_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goRush() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::RUSH && state != EState::RUSH_MOTION) {
    state = EState::RUSH_INTERMEDIATE;
    rotational_target_position = m_rotational_rush_position;
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goAllianceStake() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::ALLIANCE_STAKE &&
      state != EState::ALLIANCE_STAKE_MOTION) {
    state = EState::ALLIANCE_STAKE_INTERMEDIATE;
    rotational_target_position =
        m_rotational_alliance_stake_intermediate_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goClimbReady() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::CLIMB_READY && state != EState::CLIMB_READY_MOTION) {
    state = EState::CLIMB_READY_MOTION;
    rotational_target_position = m_rotational_climb_position;
    linear_target_position = m_linear_climb_ready_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goClimb() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::CLIMB && state != EState::CLIMB_MOTION) {
    state = EState::CLIMB_MOTION;
    rotational_target_position = m_rotational_climb_position;
    linear_target_position = m_linear_climb_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goPrevious() {
  switch (previous_state) {
    case EState::NEUTRAL:
      goNeutral();
      break;
    case EState::LOAD:
      goLoad();
      break;
    case EState::READY:
      goReady();
      break;
    case EState::RUSH:
      goRush();
      break;
  }
}

bool PIDArmMotion::isAtNeutral() { return (state == EState::NEUTRAL); }

bool PIDArmMotion::isGoingNeutral() {
  return (state == EState::NEUTRAL_MOTION);
}

bool PIDArmMotion::isAtLoad() { return (state == EState::LOAD); }

bool PIDArmMotion::isGoingLoad() {
  return (state == EState::LOAD_MOTION || state == EState::LOAD_INTERMEDIATE);
}

bool PIDArmMotion::isAtReady() { return (state == EState::READY); }

bool PIDArmMotion::isGoingReady() {
  return (state == EState::READY_MOTION || state == EState::READY_INTERMEDIATE);
}

bool PIDArmMotion::isAtScore() { return (state == EState::SCORE); }

bool PIDArmMotion::isGoingScore() {
  return (state == EState::SCORE_MOTION || state == EState::SCORE_INTERMEDIATE);
}

bool PIDArmMotion::isAtRush() { return (state == EState::RUSH); }

bool PIDArmMotion::isGoingRush() {
  return (state == EState::RUSH_MOTION || state == EState::RUSH_INTERMEDIATE);
}

bool PIDArmMotion::isAtAllianceStake() {
  return (state == EState::ALLIANCE_STAKE);
}

bool PIDArmMotion::isGoingAllianceStake() {
  return (state == EState::ALLIANCE_STAKE_INTERMEDIATE ||
          state == EState::ALLIANCE_STAKE_MOTION);
}

bool PIDArmMotion::isAtClimbReady() { return (state == EState::CLIMB_READY); }

bool PIDArmMotion::isAtClimb() { return (state == EState::CLIMB); }

void PIDArmMotion::setClock(
    const std::unique_ptr<driftless::rtos::IClock>& clock) {
  m_clock = clock->clone();
}

void PIDArmMotion::setDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDArmMotion::setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDArmMotion::setTask(std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDArmMotion::setRotationSensor(
    std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
}

void PIDArmMotion::setPotentiometer(
    std::unique_ptr<driftless::io::IPotentiometer>& potentiometer) {
  m_potentiometer = std::move(potentiometer);
}

void PIDArmMotion::setRotationMotors(
    driftless::hal::MotorGroup& rotation_motors) {
  m_rotation_motors = rotation_motors;
}

void PIDArmMotion::setLinearMotors(driftless::hal::MotorGroup& linear_motors) {
  m_linear_motors = linear_motors;
}

void PIDArmMotion::setRotationalPID(driftless::control::PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDArmMotion::setLinearPID(driftless::control::PID linear_pid) {
  m_linear_pid = linear_pid;
}

void PIDArmMotion::setRotationalNeutralPosition(
    double rotational_neutral_position) {
  m_rotational_neutral_position = rotational_neutral_position;
}

void PIDArmMotion::setRotationalLoadPosition(double rotational_load_position) {
  m_rotational_load_position = rotational_load_position;
}

void PIDArmMotion::setRotationalReadyPosition(
    double rotational_ready_position) {
  m_rotational_ready_position = rotational_ready_position;
}

void PIDArmMotion::setRotationalScorePosition(
    double rotational_score_position) {
  m_rotational_score_position = rotational_score_position;
}

void PIDArmMotion::setRotationalRushPosition(double rotational_rush_position) {
  m_rotational_rush_position = rotational_rush_position;
}

void PIDArmMotion::setRotationalAllianceStakePosition(
    double rotational_alliance_stake_position) {
  m_rotational_alliance_stake_position = rotational_alliance_stake_position;
}

void PIDArmMotion::setRotationalClimbPosition(
    double rotational_climb_position) {
  m_rotational_climb_position = rotational_climb_position;
}

void PIDArmMotion::setRotationalReadyIntermediatePosition(
    double rotational_ready_intermediate_position) {
  m_rotational_ready_intermediate_position =
      rotational_ready_intermediate_position;
}

void PIDArmMotion::setRotationalScoreIntermediatePosition(
    double rotational_score_intermediate_position) {
  m_rotational_score_intermediate_position =
      rotational_score_intermediate_position;
}

void PIDArmMotion::setRotationalRushIntermediatePosition(
    double rotational_rush_intermediate_position) {
  m_rotational_rush_intermediate_position =
      rotational_rush_intermediate_position;
}

void PIDArmMotion::setRotationalAllianceStakeIntermediatePosition(
    double rotational_alliance_stake_intermediate_position) {
  m_rotational_alliance_stake_intermediate_position =
      rotational_alliance_stake_intermediate_position;
}

void PIDArmMotion::setRotationalTolerance(double rotational_tolerance) {
  m_rotational_tolerance = rotational_tolerance;
}

void PIDArmMotion::setLinearNeutralPosition(double linear_neutral_position) {
  m_linear_neutral_position = linear_neutral_position;
}

void PIDArmMotion::setLinearLoadPosition(double linear_load_position) {
  m_linear_load_position = linear_load_position;
}

void PIDArmMotion::setLinearReadyPosition(double linear_ready_position) {
  m_linear_ready_position = linear_ready_position;
}

void PIDArmMotion::setLinearScorePosition(double linear_score_position) {
  m_linear_score_position = linear_score_position;
}

void PIDArmMotion::setLinearRushPosition(double linear_rush_position) {
  m_linear_rush_position = linear_rush_position;
}

void PIDArmMotion::setLinearAllianceStakePosition(
    double linear_alliance_stake_position) {
  m_linear_alliance_stake_position = linear_alliance_stake_position;
}

void PIDArmMotion::setLinearClimbReadyPosition(
    double linear_climb_ready_position) {
  m_linear_climb_ready_position = linear_climb_ready_position;
}

void PIDArmMotion::setLinearClimbPosition(double linear_climb_position) {
  m_linear_climb_position = linear_climb_position;
}

void PIDArmMotion::setLinearTolerance(double linear_tolerance) {
  m_linear_tolerance = linear_tolerance;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless