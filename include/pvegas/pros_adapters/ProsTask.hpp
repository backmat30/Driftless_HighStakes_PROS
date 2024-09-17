#include "pros/rtos.hpp"
#include "pvegas/rtos/ITask.hpp"
#include <memory>

namespace pvegas {
namespace pros_adapters {
class ProsTask : public rtos::ITask {
private:
  std::unique_ptr<pros::Task> task{};

public:
  // starts a task using the provided function and given parameters
  void start(void (*function)(void *), void *params) override;

  // remove the task from the scheduler
  void remove() override;

  // suspends the task
  void suspend() override;

  // resumes the task
  void resume() override;

  // pauses other tasks until this task is done
  void join() override;
};
} // namespace pros_adapters
} // namespace pvegas