#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
DirectIntakeBuilder* DirectIntakeBuilder::withMotor(
    std::unique_ptr<driftless::io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withClock(
    const std::unique_ptr<rtos::IClock>& clock) {
  m_clock = clock->clone();
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

std::unique_ptr<DirectIntake> DirectIntakeBuilder::build() {
  std::unique_ptr<DirectIntake> intake{std::make_unique<DirectIntake>()};
  intake->setMotors(m_motors);
  intake->setClock(m_clock);
  intake->setDelayer(m_delayer);
  intake->setMutex(m_mutex);
  intake->setTask(m_task);

  return intake;
}
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless