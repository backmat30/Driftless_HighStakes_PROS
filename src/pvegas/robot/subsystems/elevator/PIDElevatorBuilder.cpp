#include "pvegas/robot/subsystems/elevator/PIDElevatorBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
PIDElevatorBuilder* PIDElevatorBuilder::withDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(m_mutex);
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withMotor(
    std::unique_ptr<driftless::io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withRotationSensor(
    std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withPID(driftless::control::PID pid) {
  m_pid = pid;
  return this;
}

PIDElevatorBuilder* PIDElevatorBuilder::withRadiansToInches(
    double radians_to_inches) {
  m_radians_to_inches = radians_to_inches;
  return this;
}

std::unique_ptr<PIDElevator> PIDElevatorBuilder::build() {
  std::unique_ptr<PIDElevator> elevator{};

  elevator->setDelayer(m_delayer);
  elevator->setMutex(m_mutex);
  elevator->setTask(m_task);
  elevator->setMotors(m_motors);
  elevator->setRotationSensor(m_rotation_sensor);
  elevator->setPID(m_pid);
  elevator->setRadiansToInches(m_radians_to_inches);

  return elevator;
}
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas