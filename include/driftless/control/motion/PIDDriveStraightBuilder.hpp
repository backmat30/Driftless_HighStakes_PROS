#ifndef __PID_DRIVE_STRAIGHT_BUILDER_HPP__
#define __PID_DRIVE_STRAIGHT_BUILDER_HPP__

#include <memory>

#include "driftless/control/motion/PIDDriveStraight.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

/// @brief Builder for the PIDDriveStraight class
/// @author Matthew Backman
class PIDDriveStraightBuilder {
 private:
  // the delayer used for the control
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used for the control
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used for the control
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the linear PID controller used for the control
  PID m_linear_pid{};

  // the rotational PID controller used for the control
  PID m_rotational_pid{};

  // the target tolerance used for the control
  double m_target_tolerance{};

  // the target velocity used for the control
  double m_target_velocity{};

 public:
  /// @brief Adds a delayer to the builder
  /// @param delayer __const std::unique_ptr<driftless::rtos::IDelayer>&__ The
  /// delayer added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<driftless::rtos::IMutex>&__ The mutex added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<driftless::rtos::ITask>&__ The task added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withTask(
      std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Adds a linear PID controller to the builder
  /// @param linear_pid __PID__ The linear PID controller added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withLinearPID(PID linear_pid);

  /// @brief Adds a rotational PID controller to the builder
  /// @param rotational_pid __PID__ The rotational PID controller added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withRotationalPID(PID rotational_pid);

  /// @brief Adds a target tolerance to the builder
  /// @param target_tolerance __double__ The target tolerance added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Adds a target velocity to the builder
  /// @param target_velocity __double__ The target velocity added
  /// @return __PIDDriveStraightBuilder*__ Pointer to the current builder
  PIDDriveStraightBuilder* withTargetVelocity(double target_velocity);

  /// @brief Builds a new PIDDriveStraight object
  /// @return __std::unique_ptr<PIDDriveStraight>__ The new PIDDriveStraight
  /// object
  std::unique_ptr<PIDDriveStraight> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif