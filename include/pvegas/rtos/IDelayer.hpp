#ifndef __I_DELAYER_HPP__
#define __I_DELAYER_HPP__

#include <memory>
namespace driftless {
namespace rtos {
class IDelayer {
 public:
  virtual std::unique_ptr<IDelayer> clone() const = 0;

  virtual void delay(uint32_t millis) = 0;

  virtual void delayUntil(uint32_t time) = 0;
};
}  // namespace rtos
}  // namespace pvegas
#endif