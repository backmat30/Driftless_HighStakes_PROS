#ifndef __PID_ELEVATOR_BUILDER_HPP__
#define __PID_ELEVATOR_BUILDER_HPP__

#include "pvegas/robot/subsystems/elevator/PIDElevator.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
class PIDElevatorBuilder {
 private:
  // the delayer used to build the elevator
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used to build the elevator
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used to build the elevator
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the motor group used to build the elevator
  driftless::hal::MotorGroup m_motors{};

  // the rotation sensor used to build the elevator
  std::unique_ptr<driftless::io::IRotationSensor> m_rotation_sensor{};

  // the pid controller used to build the elevator
  driftless::control::PID m_pid{};

  // the ratio of radians to inches used to build the elevator
  double m_radians_to_inches{};

 public:
  // adds a delayer to the builder
  PIDElevatorBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // adds a mutex to the builder
  PIDElevatorBuilder* withMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // adds a task to the builder
  PIDElevatorBuilder* withTask(std::unique_ptr<driftless::rtos::ITask>& task);

  // adds a motor group to the builder
  PIDElevatorBuilder* withMotor(std::unique_ptr<driftless::io::IMotor>& motor);

  // adds a rotation sensor to the builder
  PIDElevatorBuilder* withRotationSensor(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor);

  // adds a pid controller to the builder
  PIDElevatorBuilder* withPID(driftless::control::PID pid);

  // adds a radians to inches ratio to the builder
  PIDElevatorBuilder* withRadiansToInches(double radians_to_inches);

  std::unique_ptr<driftless::robot::subsystems::elevator::PIDElevator> build();
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif