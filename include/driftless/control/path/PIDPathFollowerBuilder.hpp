#ifndef __PID_PATH_FOLLOWER_BUILDER_HPP__
#define __PID_PATH_FOLLOWER_BUILDER_HPP__

#include "driftless/control/path/PIDPathFollower.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Builder for the PIDPathFollower
/// @author Matthew Backman
class PIDPathFollowerBuilder {
 private:
  // the delayer used in the path follower
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used in the path follower
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used in the path follower
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the linear PID controller used in the path follower
  driftless::control::PID m_linear_pid{};

  // the rotational PID controller used in the path follower
  driftless::control::PID m_rotational_pid{};

  // the follow distance used in the path follower
  double m_follow_distance{};

  // the target tolerance used in the path follower
  double m_target_tolerance{};

  // the target velocity used in the path follower
  double m_target_velocity{};

 public:
  /// @brief Adds a delayer to the builder
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withDelayer(
      std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withTask(
      std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Adds a linear PID controller to the builder
  /// @param linear_pid __PID__ The linear PID controller to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withLinearPID(driftless::control::PID linear_pid);

  /// @brief Adds a rotational PID controller to the builder
  /// @param rotational_pid __PID__ The rotational PID controller to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withRotationalPID(
      driftless::control::PID rotational_pid);

  /// @brief Adds a follow distance to the builder
  /// @param follow_distance __double__ The follow distance to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withFollowDistance(double follow_distance);

  /// @brief Adds a target tolerance to the builder
  /// @param target_tolerance __double__ The target tolerance to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Adds a target velocity to the builder
  /// @param target_velocity __double__ The target velocity to add
  /// @return __PIDPathFollowerBuilder*__ Pointer to the current builder
  PIDPathFollowerBuilder* withTargetVelocity(double target_velocity);

  /// @brief Builds a new PID path follower
  /// @return __std::unique_ptr<PIDPathFollower>__ Pointer to the new PID path
  /// follower
  std::unique_ptr<PIDPathFollower> build();
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif