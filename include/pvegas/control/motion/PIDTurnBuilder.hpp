#ifndef __PID_TURN_BUILDER_HPP__
#define __PID_TURN_BUILDER_HPP__

#include "pvegas/control/motion/PIDTurn.hpp"

namespace pvegas {
namespace control {
namespace motion {
class PIDTurnBuilder {
 private:
  // the delayer used to build the control
  std::unique_ptr<pvegas::rtos::IDelayer> m_delayer{};

  // the mutex used to build the control
  std::unique_ptr<pvegas::rtos::IMutex> m_mutex{};

  // the task used to build the control
  std::unique_ptr<pvegas::rtos::ITask> m_task{};

  // the rotational PID controller used to build the control
  PID m_rotational_pid{};

  // the target tolerance used to build the control
  double m_target_tolerance{};

  // the target velocity used to build the control
  double m_target_velocity{};

 public:
  // add a delayer to the builder
  PIDTurnBuilder* withDelayer(
      const std::unique_ptr<pvegas::rtos::IDelayer>& delayer);

  // adds a mutex to the builder
  PIDTurnBuilder* withMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex);

  // adds a task to the builder
  PIDTurnBuilder* withTask(std::unique_ptr<pvegas::rtos::ITask>& task);

  // adds a rotational PID controller to the builder
  PIDTurnBuilder* withRotationalPID(PID rotational_pid);

  // adds a target tolerance to the builder
  PIDTurnBuilder* withTargetTolerance(double target_tolerance);

  // adds a target velocity to the builder
  PIDTurnBuilder* withTargetVelocity(double target_velocity);

  // builds a PIDTurn object
  std::unique_ptr<PIDTurn> build();
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif