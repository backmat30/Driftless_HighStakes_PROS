#include "pvegas/robot/subsystems/elevator/PIDElevator.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace elevator {
void PIDElevator::taskLoop(void* params) {
  PIDElevator* pid_elevator{static_cast<PIDElevator*>(params)};

  while (true) {
    pid_elevator->taskUpdate();
  }
}

void PIDElevator::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (!manual_control) {
    updatePosition();
  }

  if (m_mutex) {
    m_mutex->give();
  }
  m_delayer->delay(TASK_DELAY);
}

void PIDElevator::updatePosition() {
  double current_position{getPosition()};
  double voltage{m_pid.getControlValue(current_position, m_position)};
  m_motors.setVoltage(voltage);
}

void PIDElevator::init() {
  m_motors.init();
  if (m_rotation_sensor) {
    m_rotation_sensor->reset();
  }
  m_pid.reset();
}

void PIDElevator::run() { m_task->start(PIDElevator::taskLoop, this); }

void PIDElevator::setVoltage(double voltage) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_motors.setVoltage(voltage);
  manual_control = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDElevator::setPosition(double position) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_position = position;
  manual_control = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

double PIDElevator::getPosition() {
  double position{};
  if (m_rotation_sensor) {
    position = m_rotation_sensor->getRotations() * m_radians_to_inches;
  } else {
    position = m_motors.getPosition() * m_radians_to_inches;
  }

  return position;
}

void PIDElevator::setDelayer(
    const std::unique_ptr<pvegas::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDElevator::setMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDElevator::setTask(std::unique_ptr<pvegas::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDElevator::setMotors(pvegas::hal::MotorGroup motors) {
  m_motors = motors;
}

void PIDElevator::setRotationSensor(
    std::unique_ptr<pvegas::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
}

void PIDElevator::setPID(pvegas::control::PID pid) { m_pid = pid; }

void PIDElevator::setRadiansToInches(double radians_to_inches) {
  m_radians_to_inches = radians_to_inches;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas