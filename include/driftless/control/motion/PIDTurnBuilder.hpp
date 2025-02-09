#ifndef __PID_TURN_BUILDER_HPP__
#define __PID_TURN_BUILDER_HPP__

#include "driftless/control/motion/PIDTurn.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion controls
/// @author Matthew Backman
namespace motion {

/// @brief Builder for the PIDTurn class
class PIDTurnBuilder {
 private:
  // the delayer used to build the control
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used to build the control
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used to build the control
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the rotational PID controller used to build the control
  PID m_rotational_pid{};

  // the target tolerance used to build the control
  double m_target_tolerance{};

  // the target velocity used to build the control
  double m_target_velocity{};

 public:
  /// @brief Adds a delayer to the builder
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer
  /// added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Adds a rotational PID controller to the builder
  /// @param rotational_pid __PID__ The rotational PID controller added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withRotationalPID(PID rotational_pid);

  /// @brief Adds a target tolerance to the builder
  /// @param target_tolerance __double__ The target tolerance added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Adds a target velocity to the builder
  /// @param target_velocity __double__ The target velocity added
  /// @return __PIDTurnBuilder*__ Pointer to the current builder
  PIDTurnBuilder* withTargetVelocity(double target_velocity);

  /// @brief Builds a new PIDTurn object
  /// @return __std::unique_ptr<PIDTurn>__ The new PIDTurn object
  std::unique_ptr<PIDTurn> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif