#ifndef __PROS_MUTEX_HPP__
#define __PROS_MUTEX_HPP__

#include "driftless/rtos/IMutex.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief The class for adapting a PROS mutex
/// @author Matthew Backman
class ProsMutex : public rtos::IMutex {
 private:
  // pros mutex being adapted
  pros::Mutex mutex{};

 public:
  /// @brief Takes the mutex, blocking other tasks from running without it
  void take() override;

  /// @brief Gives the mutex back, unblocking it for other tasks
  void give() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif