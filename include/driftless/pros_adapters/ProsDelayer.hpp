#ifndef __PROS_DELAYER_HPP__
#define __PROS_DELAYER_HPP__

#include <cstdint>
#include <memory>

#include "pros/rtos.hpp"
#include "driftless/rtos/IDelayer.hpp"

namespace driftless {
namespace pros_adapters {
class ProsDelayer : public rtos::IDelayer {
 public:
  std::unique_ptr<rtos::IDelayer> clone() const override;

  void delay(uint32_t millis) override;

  void delayUntil(uint32_t time) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif