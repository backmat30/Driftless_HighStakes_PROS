#ifndef __I_MUTEX_HPP__
#define __I_MUTEX_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for real-time operating system code
/// @author Matthew Backman
namespace rtos {
/// @brief Interface for a mutex object
/// @author Matthew Backman
class IMutex {
 public:
  /// @brief Deletes the mutex object
  virtual ~IMutex() = default;

  /// @brief Takes the mutex
  virtual void take() = 0;

  /// @brief Gives the mutex
  virtual void give() = 0;
};
}  // namespace rtos
}  // namespace driftless
#endif