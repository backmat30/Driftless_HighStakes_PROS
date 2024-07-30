#ifndef __PROS_DELAYER_HPP__
#define __PROS_DELAYER_HPP__

#include "pros/rtos.hpp"

#include "pvegas/rtos/IDelayer.hpp"

#include <cstdint>
#include <memory>

namespace pvegas {
namespace pros_adapters {
class ProsDelayer : public rtos::IDelayer {
public:
  std::unique_ptr<rtos::IDelayer> clone() const override;

  void delay(uint32_t millis) override;

  void delayUntil(uint32_t time) override;
};
} // namespace pros_adapters
} // namespace pvegas
#endif