#include "pvegas/robot/subsystems/arm/PIDArmMotion.hpp"

namespace pvegas {
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
  updateState();
  updatePosition();

  if (m_mutex) {
    m_mutex->give();
  }
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
      if (std::abs(rotation_position - m_rotational_score_position) <=
              m_rotational_tolerance &&
          std::abs(linear_position - m_linear_score_position) <=
              m_linear_tolerance) {
        state = EState::SCORE;
      }
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
  if (state != EState::LOAD) {
    state = EState::LOAD_MOTION;
    rotational_target_position = m_rotational_load_position;
    linear_target_position = m_linear_load_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goReady() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::READY) {
    state = EState::READY_MOTION;
    rotational_target_position = m_rotational_ready_position;
    linear_target_position = m_linear_ready_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDArmMotion::goScore() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (state != EState::SCORE) {
    state = EState::SCORE_MOTION;
    rotational_target_position = m_rotational_score_position;
    linear_target_position = m_linear_score_position;
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDArmMotion::isAtNeutral() { return (state == EState::NEUTRAL); }

bool PIDArmMotion::isGoingNeutral() {
  return (state == EState::NEUTRAL_MOTION);
}

bool PIDArmMotion::isAtLoad() { return (state == EState::LOAD); }

bool PIDArmMotion::isGoingLoad() { return (state == EState::LOAD_MOTION); }

bool PIDArmMotion::isAtReady() { return (state == EState::READY); }

bool PIDArmMotion::isGoingReady() { return (state == EState::READY_MOTION); }

bool PIDArmMotion::isAtScore() { return (state == EState::SCORE); }

bool PIDArmMotion::isGoingScore() { return (state == EState::SCORE_MOTION); }

void PIDArmMotion::setDelayer(
    const std::unique_ptr<pvegas::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDArmMotion::setMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDArmMotion::setTask(std::unique_ptr<pvegas::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDArmMotion::setRotationSensor(
    std::unique_ptr<pvegas::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
}

void PIDArmMotion::setPotentiometer(
    std::unique_ptr<pvegas::io::IPotentiometer>& potentiometer) {
  m_potentiometer = std::move(potentiometer);
}

void PIDArmMotion::setRotationMotors(pvegas::hal::MotorGroup& rotation_motors) {
  m_rotation_motors = rotation_motors;
}

void PIDArmMotion::setLinearMotors(pvegas::hal::MotorGroup& linear_motors) {
  m_linear_motors = linear_motors;
}

void PIDArmMotion::setRotationalPID(pvegas::control::PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDArmMotion::setLinearPID(pvegas::control::PID linear_pid) {
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

void PIDArmMotion::setLinearTolerance(double linear_tolerance) {
  m_linear_tolerance = linear_tolerance;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas