#ifndef __PROS_CLOCK_HPP__
#define __PROS_CLOCK_HPP__

#include <cstdint>
#include <memory>

#include "pros/rtos.hpp"
#include "driftless/rtos/IClock.hpp"

namespace driftless {
namespace pros_adapters {
class ProsClock : public rtos::IClock {
 public:
  std::unique_ptr<rtos::IClock> clone() const override;

  uint32_t getTime() override;
};
}  // namespace pros_adapters
}  // namespace driftless

#endif