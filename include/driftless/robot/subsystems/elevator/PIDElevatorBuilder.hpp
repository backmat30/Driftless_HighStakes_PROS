#ifndef __PID_ELEVATOR_BUILDER_HPP__
#define __PID_ELEVATOR_BUILDER_HPP__

#include "driftless/robot/subsystems/elevator/PIDElevator.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for elevator subsystem code
/// @author Matthew Backman
namespace elevator {

/// @brief Builder class for creating PIDElevator objects
/// @author Matthew Backman
class PIDElevatorBuilder {
 private:
  std::unique_ptr<rtos::IClock> m_clock{};

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
  /// @brief sets the clock for the builder
  /// @param clock __const std::unique_ptr<rtos::IClock>&__ The clock to use
  /// @return __PIDElevatorBuilder*__ Pointer to the builder
  PIDElevatorBuilder* withClock(const std::unique_ptr<rtos::IClock>& clock);

  // adds a delayer to the builder
  PIDElevatorBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // adds a mutex to the builder
  PIDElevatorBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

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

  /// @brief Builds a new PIDElevator object
  /// @return __std::unique_ptr<PIDElevator>__ Pointer to the new PIDElevator
  /// object
  std::unique_ptr<driftless::robot::subsystems::elevator::PIDElevator> build();
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif