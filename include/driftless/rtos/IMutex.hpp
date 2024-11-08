#ifndef __I_MUTEX_HPP__
#define __I_MUTEX_HPP__

namespace driftless {
namespace rtos {
class IMutex {
 public:
  virtual ~IMutex() = default;

  // takes the mutex, blocking other tasks from running without it
  virtual void take() = 0;

  // gives the mutex back, unblocking it for other tasks
  virtual void give() = 0;
};
}  // namespace rtos
}  // namespace driftless
#endif