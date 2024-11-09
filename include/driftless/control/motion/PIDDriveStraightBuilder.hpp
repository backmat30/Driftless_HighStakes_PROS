#ifndef __PID_DRIVE_STRAIGHT_BUILDER_HPP__
#define __PID_DRIVE_STRAIGHT_BUILDER_HPP__

#include <memory>

#include "driftless/control/motion/PIDDriveStraight.hpp"

namespace driftless {
namespace control {
namespace motion {
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
  // adds a delayer to the builder
  PIDDriveStraightBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // adds a mutex to the builder
  PIDDriveStraightBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // adds a task to the builder
  PIDDriveStraightBuilder* withTask(
      std::unique_ptr<driftless::rtos::ITask>& task);

  // adds a linear PID controller to the builder
  PIDDriveStraightBuilder* withLinearPID(PID linear_pid);

  // adds a rotational PID controller to the builder
  PIDDriveStraightBuilder* withRotationalPID(PID rotational_pid);

  // adds a target tolerance to the builder
  PIDDriveStraightBuilder* withTargetTolerance(double target_tolerance);

  // adds a target velocity to the builder
  PIDDriveStraightBuilder* withTargetVelocity(double target_velocity);

  // build a PIDDriveStraight object
  std::unique_ptr<PIDDriveStraight> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif