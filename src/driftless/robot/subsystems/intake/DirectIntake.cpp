#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
void DirectIntake::taskLoop(void* params) {
  DirectIntake* direct_intake{static_cast<DirectIntake*>(params)};

  while (true) {
    direct_intake->taskUpdate();
  }
}

void DirectIntake::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  unjam();

  if (m_mutex) {
    m_mutex->give();
  }
  m_delayer->delay(TASK_DELAY);
}

void DirectIntake::unjam() {
  if (m_motors.getEfficiency() < 0.2) {
    if (!jammed) {
      jammed = true;
      m_motors.setVoltage(-12.0);
      latest_jam_time = m_clock->getTime();
    }
  }
  if (jammed && m_clock->getTime() > latest_jam_time + 500) {
    jammed = false;
    m_motors.setVoltage(target_voltage);
  }
}

void DirectIntake::init() { m_motors.init(); }

void DirectIntake::run() { m_task->start(taskLoop, this); }

void DirectIntake::setVoltage(double voltage) {
  if (m_mutex) {
    m_mutex->take();
  }

  target_voltage = voltage;
  if (!jammed) {
    m_motors.setVoltage(target_voltage);
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::setMotors(driftless::hal::MotorGroup& motors) {
  m_motors = motors;
}

void DirectIntake::setClock(const std::unique_ptr<rtos::IClock>& clock) {
  m_clock = clock->clone();
}

void DirectIntake::setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void DirectIntake::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void DirectIntake::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless