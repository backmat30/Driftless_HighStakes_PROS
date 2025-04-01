#ifndef __PID_BOOMERANG_BUILDER_HPP__
#define __PID_BOOMERANG_BUILDER_HPP__

#include "driftless/control/boomerang/PIDBoomerang.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the boomerang control
/// @author Matthew Backman
namespace boomerang {

class PIDBoomerangBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  PID m_linear_pid{};

  PID m_rotational_pid{};

  double m_lead{};

  double m_aim_distance{};

  double m_target_tolerance{};

  double m_target_velocity{};

 public:
  /// @brief Adds a delayer to the builder
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer to
  /// add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withDelayer(
      const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Adds a linear PID controller to the builder
  /// @param linear_pid __PID__ The linear PID controller to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withLinearPID(PID linear_pid);

  /// @brief Adds a rotational PID controller to the builder
  /// @param rotational_pid __PID__ The rotational PID controller to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withRotationalPID(PID rotational_pid);

  /// @brief Adds a lead distance to the builder
  /// @param lead __double__ The lead distance to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withLead(double lead);

  /// @brief Adds an aim distance to the builder
  /// @param aim_distance __double__ The aim distance to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withAimDistance(double aim_distance);

  /// @brief Adds a target tolerance to the builder
  /// @param target_tolerance __double__ The target tolerance to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Adds a target velocity to the builder
  /// @param target_velocity __double__ The target velocity to add
  /// @return __PIDBoomerangBuilder*__ Pointer to the current builder
  PIDBoomerangBuilder* withTargetVelocity(double target_velocity);

  /// @brief Build a new PIDBoomerang object
  /// @return __std::unique_ptr<PIDBoomerang>__ The new PIDBoomerang object
  std::unique_ptr<PIDBoomerang> build();
};
}  // namespace boomerang
}  // namespace control
}  // namespace driftless
#endif