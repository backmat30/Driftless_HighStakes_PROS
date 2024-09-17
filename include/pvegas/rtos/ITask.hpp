#ifndef __I_TASK_HPP__
#define __I_TASK_HPP__

namespace pvegas {
namespace rtos {
class ITask {
public:
  virtual ~ITask() = 0;

  // starts a task using the provided function and given parameters
  virtual void start(void (*function)(void *), void *params) = 0;

  // remove the task from the scheduler
  virtual void remove() = 0;

  // suspends the task
  virtual void suspend() = 0;

  // resumes the task
  virtual void resume() = 0;

  // pauses other tasks until this task is done
  virtual void join() = 0;
};
} // namespace rtos
} // namespace pvegas
#endif