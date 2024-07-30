#ifndef __PROS_CLOCK_HPP__
#define __PROS_CLOCK_HPP__

#include "pros/rtos.hpp"

#include "pvegas/rtos/IClock.hpp"

#include <cstdint>
#include <memory>

namespace pvegas {
namespace pros_adapters {
class ProsClock : public rtos::IClock {
public:
  std::unique_ptr<rtos::IClock> clone() const override;

  uint32_t getTime() override;
};
} // namespace pros_adapters
} // namespace pvegas

#endif