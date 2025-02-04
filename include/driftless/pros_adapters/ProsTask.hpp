#ifndef __PROS_TASK_HPP__
#define __PROS_TASK_HPP__

#include <memory>

#include "driftless/rtos/ITask.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Class to adapt the pros task class to the ITask interface
/// @author Matthew Backman
class ProsTask : public rtos::ITask {
 private:
  std::unique_ptr<pros::Task> task{};

 public:
  /// @brief Starts a new task
  /// @param function __void (*)(void*)__ The function callback ran by the task
  /// @param params __void*__ Potential parameters of the given function
  void start(void (*function)(void *), void *params) override;

  /// @brief Removes the task
  void remove() override;

  /// @brief Suspends the task
  void suspend() override;

  /// @brief Resumes the task
  void resume() override;

  /// @brief Joins the task
  void join() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif