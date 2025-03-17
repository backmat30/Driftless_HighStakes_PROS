#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

  /// @brief The namespace for intake subsystem code
/// @author Matthew Backman
namespace intake {

/// @brief Builder class for creating DirectIntake objects
/// @author Matthew Backman
class DirectIntakeBuilder {
 private:
  // the motors used for the intake
  driftless::hal::MotorGroup m_motors{};

  std::unique_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

 public:
  /// @brief Adds motors to the builder
  /// @param motor __std::unique_ptr<driftless::io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withMotor(std::unique_ptr<driftless::io::IMotor>& motor);

  /// @brief Adds a clock to the builder
  /// @param clock __const std::unique_ptr<rtos::IClock>&__ The clock to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withClock(const std::unique_ptr<rtos::IClock>& clock);
  
  /// @brief Adds a delayer to the builder
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withDelayer(const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Builds a new DirectIntake object
  /// @return __std::unique_ptr<DirectIntake>__ The built DirectIntake object
  std::unique_ptr<DirectIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif