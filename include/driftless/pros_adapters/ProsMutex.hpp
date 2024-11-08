#ifndef __PROS_MUTEX_HPP__
#define __PROS_MUTEX_HPP__

#include "pros/rtos.hpp"
#include "driftless/rtos/IMutex.hpp"
namespace driftless {
namespace pros_adapters {
class ProsMutex : public rtos::IMutex {
 private:
  // pros mutex being adapted
  pros::Mutex mutex{};

 public:
  // takes the mutex, blocking other tasks from running without it
  void take() override;

  // gives the mutex back, unblocking it for other tasks
  void give() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif