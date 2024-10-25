#ifndef __PID_GO_TO_POINT_BUILDER_HPP__
#define __PID_GO_TO_POINT_BUILDER_HPP__

#include "pvegas/control/motion/PIDGoToPoint.hpp"

namespace pvegas {
namespace control {
namespace motion {
class PIDGoToPointBuilder {
 private:
  // the delayer used to build the control
  std::unique_ptr<pvegas::rtos::IDelayer> m_delayer{};

  // the mutex used to build the control
  std::unique_ptr<pvegas::rtos::IMutex> m_mutex{};

  // the task used to build the control
  std::unique_ptr<pvegas::rtos::ITask> m_task{};

  // the linear PID controller used to build the control
  PID m_linear_pid{};

  // the rotational PID controller used to build the control
  PID m_rotational_pid{};

  // the target tolerance used to build the control
  double m_target_tolerance{};

  // the target velocity used to build the control
  double m_target_velocity{};

 public:
  // add a delayer to the builder
  PIDGoToPointBuilder* withDelayer(
      const std::unique_ptr<pvegas::rtos::IDelayer>& delayer);

  // add a mutex to the builder
  PIDGoToPointBuilder* withMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex);

  // add a task to the builder
  PIDGoToPointBuilder* withTask(std::unique_ptr<pvegas::rtos::ITask>& task);

  // add a linear pid controller to the builder
  PIDGoToPointBuilder* withLinearPID(PID linear_pid);

  // add a rotational pid controller to the builder
  PIDGoToPointBuilder* withRotationalPID(PID rotational_pid);

  // add a target tolerance to the builder
  PIDGoToPointBuilder* withTargetTolerance(double target_tolerance);

  // add a target velocity to the builder
  PIDGoToPointBuilder* withTargetVelocity(double target_velocity);

  // build a PIDGoToPoint object
  std::unique_ptr<PIDGoToPoint> build();
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif