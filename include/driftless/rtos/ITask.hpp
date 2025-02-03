#ifndef __I_TASK_HPP__
#define __I_TASK_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Namespace for real-time operating system code
/// @author Matthew Backman
namespace rtos {
/// @brief Interface for a task in the real-time operating system
/// @author Matthew Backman
class ITask {
 public:
  /// @brief Deletes the task object
  virtual ~ITask() = default;

  /// @brief Starts the task
  /// @param function __void (*)(void *)__ The function to run in the task
  /// @param params __void *__ The parameters to pass to the function
  virtual void start(void (*function)(void *), void *params) = 0;

  /// @brief Removes the task from the scheduler
  virtual void remove() = 0;

  /// @brief Suspends the task
  virtual void suspend() = 0;

  /// @brief Resumes the task
  virtual void resume() = 0;

  /// @brief Joins the task
  virtual void join() = 0;
};
}  // namespace rtos
}  // namespace driftless
#endif