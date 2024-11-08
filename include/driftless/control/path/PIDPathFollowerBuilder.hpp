#ifndef __PID_PATH_FOLLOWER_BUILDER_HPP__
#define __PID_PATH_FOLLOWER_BUILDER_HPP__

#include "driftless/control/path/PIDPathFollower.hpp"

namespace driftless {
namespace control {
namespace path {
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
  // add a delayer to the builder
  PIDPathFollowerBuilder* withDelayer(
      std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // add a mutex to the builder
  PIDPathFollowerBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // add a task to the builder
  PIDPathFollowerBuilder* withTask(std::unique_ptr<driftless::rtos::ITask>& task);

  // add a linear pid controller to the builder
  PIDPathFollowerBuilder* withLinearPID(driftless::control::PID linear_pid);

  // add a rotational pid controller to the builder
  PIDPathFollowerBuilder* withRotationalPID(
      driftless::control::PID rotational_pid);

  // add a follow distance to the builder
  PIDPathFollowerBuilder* withFollowDistance(double follow_distance);

  // add a target tolerance to the builder
  PIDPathFollowerBuilder* withTargetTolerance(double target_tolerance);

  // add a target velocity to the builder
  PIDPathFollowerBuilder* withTargetVelocity(double target_velocity);

  // build the path follower
  std::unique_ptr<PIDPathFollower> build();
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif